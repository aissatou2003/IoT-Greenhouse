#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "driver/uart.h"

// NimBLE
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "host/ble_gap.h"
#include "host/ble_hs_adv.h"   // ble_hs_adv_parse_fields
#include "host/ble_uuid.h"
#include "os/os_mbuf.h"

// -------------------- UART (uplink fiable avec ACK;SEQ=) --------------------
#define UART_PORT UART_NUM_0
#define UART_BUF  256

static const char *TAG = "S3_GATEWAY";

// ACK attendu = "ACK;SEQ=%d"
static bool is_expected_ack(const char *line, int seq)
{
    char expected[32];
    snprintf(expected, sizeof(expected), "ACK;SEQ=%d", seq);
    return (strncmp(line, expected, strlen(expected)) == 0);
}

/**
 * Envoie une ligne sur UART et attend un ACK correspondant au SEQ (max timeout_us).
 * Retourne true si ACK OK, false sinon.
 */
static bool uart_send_and_wait_ack(const char *line_to_send, int seq, int64_t timeout_us)
{
    uint8_t rx[UART_BUF];

    uart_write_bytes(UART_PORT, line_to_send, strlen(line_to_send));
    ESP_LOGI(TAG, "UART TX: %s", line_to_send);

    int64_t deadline = esp_timer_get_time() + timeout_us;

    while (esp_timer_get_time() < deadline)
    {
        int len = uart_read_bytes(UART_PORT, rx, UART_BUF - 1, 200 / portTICK_PERIOD_MS);
        if (len <= 0) continue;

        rx[len] = 0;
        char *s = (char *)rx;

        // clean CRLF
        for (int i = 0; s[i]; i++)
        {
            if (s[i] == '\r' || s[i] == '\n') { s[i] = 0; break; }
        }

        ESP_LOGI(TAG, "UART RX: %s", s);

        if (strncmp(s, "ACK;SEQ=", 8) != 0) continue;

        if (is_expected_ack(s, seq))
        {
            ESP_LOGI(TAG, "ACK valide ✅ (SEQ=%d)", seq);
            return true;
        }
        else
        {
            ESP_LOGW(TAG, "ACK d’un autre SEQ (ignoré). Attendu SEQ=%d", seq);
        }
    }

    ESP_LOGW(TAG, "Timeout: pas d'ACK valide pour SEQ=%d", seq);
    return false;
}

static void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(UART_PORT, UART_BUF * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_flush_input(UART_PORT);
}

// -------------------- BLE (NimBLE Central) --------------------

// UUIDs (doivent matcher côté C6)
static const ble_uuid128_t SVC_UUID =
    BLE_UUID128_INIT(0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x89, 0x67, 0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static const ble_uuid128_t CHR_UUID =
    BLE_UUID128_INIT(0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x89, 0x67, 0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

// Etat BLE
static uint16_t g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool g_subscribed = false;

// Handles GATT trouvés
static uint16_t g_svc_start = 0, g_svc_end = 0;
static uint16_t g_chr_val_handle = 0;
static uint16_t g_cccd_handle = 0;

// Pour SEQ
static int g_seq = 1;

// ✅ IMPORTANT: le bon addr_type (public/random) est fourni par ble_hs_id_infer_auto()
static uint8_t g_own_addr_type = BLE_OWN_ADDR_PUBLIC;

// Utilitaire: check name starts with "C6_"
static bool adv_name_starts_with_c6(const struct ble_hs_adv_fields *f)
{
    if (!f->name || f->name_len < 3) return false;
    return (f->name[0] == 'C' && f->name[1] == '6' && f->name[2] == '_');
}

static void ble_scan_start(void);

// Abonnement notifications: écrit 0x0001 dans le CCCD
static int subscribe_to_notifications(void)
{
    if (g_cccd_handle == 0)
    {
        ESP_LOGE(TAG, "CCCD handle inconnu, impossible de subscribe");
        return -1;
    }

    ESP_LOGI(TAG, "Écriture CCCD pour activer NOTIFY (handle=%u)", g_cccd_handle);

    uint8_t cccd[2] = {0x01, 0x00}; // notifications enable

    int rc = ble_gattc_write_flat(g_conn_handle, g_cccd_handle, cccd, sizeof(cccd), NULL, NULL);
    if (rc == 0)
    {
        g_subscribed = true;
        ESP_LOGI(TAG, "Subscribed ✅ (CCCD write ok)");
    }
    else
    {
        ESP_LOGE(TAG, "Subscribe failed rc=%d", rc);
    }
    return rc;
}

// Découverte CCCD après avoir trouvé la characteristic
static int desc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                        uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg)
{
    (void)conn_handle;
    (void)chr_val_handle;
    (void)arg;

    if (error->status == 0 && dsc)
    {
        // CCCD UUID = 0x2902
        if (ble_uuid_u16((const ble_uuid_t *)&dsc->uuid) == 0x2902)
        {
            g_cccd_handle = dsc->handle;
            ESP_LOGI(TAG, "CCCD trouvé handle=%u", g_cccd_handle);
        }
        return 0;
    }

    // Fin de découverte des descripteurs
    if (g_cccd_handle != 0 && !g_subscribed)
    {
        subscribe_to_notifications();
    }
    else if (g_cccd_handle == 0)
    {
        ESP_LOGW(TAG, "Pas de CCCD trouvé -> pas de notifications possible");
    }
    return 0;
}

// Découverte characteristic dans le service
static int chr_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg)
{
    (void)arg;

    if (error->status == 0 && chr)
    {
        if (ble_uuid_cmp((const ble_uuid_t *)&chr->uuid, &CHR_UUID.u) == 0)
        {
            g_chr_val_handle = chr->val_handle;
            ESP_LOGI(TAG, "Telemetry characteristic trouvée val_handle=%u", g_chr_val_handle);
        }
        return 0;
    }

    if (g_chr_val_handle == 0)
    {
        ESP_LOGE(TAG, "Characteristic telemetry introuvable (UUID mismatch ?)");
        return 0;
    }

    // Découvrir CCCD pour activer notify
    int rc = ble_gattc_disc_all_dscs(conn_handle, g_chr_val_handle, g_svc_end, desc_disc_cb, NULL);
    if (rc != 0) ESP_LOGE(TAG, "disc dscs rc=%d", rc);
    return 0;
}

// Découverte service
static int svc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *svc, void *arg)
{
    (void)arg;

    if (error->status == 0 && svc)
    {
        if (ble_uuid_cmp((const ble_uuid_t *)&svc->uuid, &SVC_UUID.u) == 0)
        {
            g_svc_start = svc->start_handle;
            g_svc_end   = svc->end_handle;
            ESP_LOGI(TAG, "Service trouvé start=%u end=%u", g_svc_start, g_svc_end);
        }
        return 0;
    }

    if (g_svc_start == 0)
    {
        ESP_LOGE(TAG, "Service introuvable (UUID mismatch ?)");
        return 0;
    }

    // Découvrir la characteristic
    int rc = ble_gattc_disc_all_chrs(conn_handle, g_svc_start, g_svc_end, chr_disc_cb, NULL);
    if (rc != 0) ESP_LOGE(TAG, "disc chrs rc=%d", rc);
    return 0;
}

// GAP event handler (scan, connect, disconnect, notify)
static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type)
    {
    case BLE_GAP_EVENT_DISC:
    {
        struct ble_hs_adv_fields fields;
        int rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
        if (rc != 0) return 0;

        if (adv_name_starts_with_c6(&fields))
        {
            char name[32] = {0};
            int l = fields.name_len < (int)sizeof(name) - 1 ? fields.name_len : (int)sizeof(name) - 1;
            memcpy(name, fields.name, l);

            ESP_LOGI(TAG, "Trouvé périphérique C6: name=%s RSSI=%d -> connexion", name, event->disc.rssi);

            ble_gap_disc_cancel();

            struct ble_gap_conn_params params = {
                .scan_itvl = 0x0010,
                .scan_window = 0x0010,
                .itvl_min = 0x0018,
                .itvl_max = 0x0028,
                .latency = 0,
                .supervision_timeout = 0x0100,
                .min_ce_len = 0x0000,
                .max_ce_len = 0x0000,
            };

            // ✅ utiliser g_own_addr_type (pas BLE_OWN_ADDR_PUBLIC en dur)
            rc = ble_gap_connect(g_own_addr_type, &event->disc.addr, 30000, &params, gap_event_cb, NULL);
            if (rc != 0)
            {
                ESP_LOGE(TAG, "connect rc=%d", rc);
                ble_scan_start();
            }
        }
        return 0;
    }

    case BLE_GAP_EVENT_CONNECT:
    {
        if (event->connect.status != 0)
        {
            ESP_LOGE(TAG, "Connexion échouée status=%d", event->connect.status);
            ble_scan_start();
            return 0;
        }

        g_conn_handle = event->connect.conn_handle;
        ESP_LOGI(TAG, "Connecté ✅ conn_handle=%u", g_conn_handle);

        int rc = ble_gattc_disc_all_svcs(g_conn_handle, svc_disc_cb, NULL);
        if (rc != 0) ESP_LOGE(TAG, "disc svcs rc=%d", rc);
        return 0;
    }

    case BLE_GAP_EVENT_DISCONNECT:
    {
        ESP_LOGW(TAG, "Déconnecté. reason=%d", event->disconnect.reason);
        g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        g_subscribed = false;
        g_svc_start = g_svc_end = 0;
        g_chr_val_handle = 0;
        g_cccd_handle = 0;
        ble_scan_start();
        return 0;
    }

    case BLE_GAP_EVENT_NOTIFY_RX:
    {
        if (!event->notify_rx.om) return 0;

        char payload[128];
        int n = OS_MBUF_PKTLEN(event->notify_rx.om);
        if (n >= (int)sizeof(payload)) n = sizeof(payload) - 1;

        ble_hs_mbuf_to_flat(event->notify_rx.om, payload, n, NULL);
        payload[n] = 0;

        ESP_LOGI(TAG, "BLE NOTIF RX: %s", payload);

        char line[200];
        snprintf(line, sizeof(line),
                 "S1;SEQ=%d;PRIO=N;TYPE=TEL;%s\n",
                 g_seq, payload);

        if (uart_send_and_wait_ack(line, g_seq, 2LL * 1000 * 1000)) g_seq++;

        return 0;
    }

    default:
        return 0;
    }
}

static void ble_scan_start(void)
{
    struct ble_gap_disc_params disc_params = {0};
    disc_params.passive = 0;
    disc_params.itvl = 0x0010;
    disc_params.window = 0x0010;
    disc_params.filter_duplicates = 1;

    // ✅ utiliser g_own_addr_type (pas BLE_OWN_ADDR_PUBLIC en dur)
    int rc = ble_gap_disc(g_own_addr_type, BLE_HS_FOREVER, &disc_params, gap_event_cb, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gap_disc rc=%d", rc);
    }
    else
    {
        ESP_LOGI(TAG, "Scan BLE démarré...");
    }
}

static void ble_on_sync(void)
{
    // ✅ FIX CRASH: ne PAS passer NULL !
    uint8_t addr_type;
    int rc = ble_hs_id_infer_auto(0, &addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed rc=%d", rc);
        return;
    }

    g_own_addr_type = addr_type;
    ESP_LOGI(TAG, "BLE addr_type=%d (own_addr_type)", g_own_addr_type);

    ble_scan_start();
}

static void ble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    uart_init();

    nimble_port_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_svc_gap_device_name_set("S3_GATEWAY");

    // ✅ Sync callback (quand la stack BLE est prête)
    ble_hs_cfg.sync_cb = ble_on_sync;

    nimble_port_freertos_init(ble_host_task);
}
