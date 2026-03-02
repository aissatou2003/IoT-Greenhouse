#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"

static const char *TAG = "S3_ORCHESTRATOR";

// UUIDs synchronisés avec le C6
static const ble_uuid16_t SVC_UUID = BLE_UUID16_INIT(0x00FF);
static const ble_uuid16_t CHR_TEMP = BLE_UUID16_INIT(0xFF01);

static uint8_t g_own_addr_type;
static int g_seq = 1;

void init_uart(void) {
    uart_config_t cfg = {
        .baud_rate = 115200, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &cfg);
}

void orchestrate_to_lora(const char *label, const char *val) {
    char final_msg[256];
    // Format S1 : Télémétrie périodique
    snprintf(final_msg, sizeof(final_msg), "S1;SEQ=%d;PRIO=N;TYPE=TEL;%s=%s\n", g_seq++, label, val);
    
    ESP_LOGI(TAG, ">>> ENVOI LORA: %s", final_msg);
    uart_write_bytes(UART_NUM_0, final_msg, strlen(final_msg));
}

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    struct ble_hs_adv_fields fields;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
        if (rc != 0) return 0;

        if (fields.name_len == 13 && strncmp((char *)fields.name, "GreenHouse-C6", 13) == 0) {
            ESP_LOGI(TAG, "Serre détectée ! Connexion...");
            ble_gap_disc_cancel();

            // Configuration manuelle des paramètres pour éviter l'erreur 530
            struct ble_gap_conn_params conn_params = {
                .scan_itvl = 0x0010,           // Intervalle de scan
                .scan_window = 0x0010,         // Fenêtre de scan
                .itvl_min = 0x0018,            // Intervalle min (30ms)
                .itvl_max = 0x0028,            // Intervalle max (50ms)
                .latency = 0,                  // Pas de latence esclave
                .supervision_timeout = 0x0100, // Timeout de 2.5 secondes
                .min_ce_len = 0,
                .max_ce_len = 0,
            };

            rc = ble_gap_connect(g_own_addr_type, &event->disc.addr, 30000, &conn_params, gap_event_cb, NULL);
            if (rc != 0) {
                ESP_LOGE(TAG, "Erreur ble_gap_connect rc=%d", rc);
            }
        }
        break;

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "Connecté à GreenHouse-C6 ✅");
        } else {
            struct ble_gap_disc_params d_params = {0};
            ble_gap_disc(g_own_addr_type, BLE_HS_FOREVER, &d_params, gap_event_cb, NULL);
        }
        break;

    case BLE_GAP_EVENT_NOTIFY_RX:
        if (event->notify_rx.om) {
            char buf[128]; // On augmente la taille pour toute la phrase
            int len = OS_MBUF_PKTLEN(event->notify_rx.om);
            
            if (len > 0) {
                // On limite à la taille de notre buffer
                if (len > sizeof(buf) - 1) len = sizeof(buf) - 1;
                
                // On copie les données reçues dans notre chaîne de texte
                ble_hs_mbuf_to_flat(event->notify_rx.om, buf, len, NULL);
                buf[len] = '\0'; // On termine proprement la chaîne

                ESP_LOGI(TAG, "Message complet reçu : %s", buf);

                // On vérifie si c'est une alerte pour la QoS [cite: 173]
                char priority = 'N';
                if (strstr(buf, "ALERTE") || strstr(buf, "CRITIQUE")) {
                    priority = 'H';
                }

                // On formate pour la LoRa au format S1 [cite: 162, 176]
                char final_msg[256];
                snprintf(final_msg, sizeof(final_msg), "S1;SEQ=%d;PRIO=%c;TYPE=TEL;%s\n", 
                         g_seq++, priority, buf);
                
                ESP_LOGI(TAG, ">>> TRANSFERT LORA : %s", final_msg);
                uart_write_bytes(UART_NUM_0, final_msg, strlen(final_msg));
            }
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "Déconnecté. Scan relancé...");
        struct ble_gap_disc_params disc_params = {0};
        ble_gap_disc(g_own_addr_type, BLE_HS_FOREVER, &disc_params, gap_event_cb, NULL);
        break;
    }
    return 0;
}

static void on_sync(void) {
    // Utilisation des variables pour éviter l'erreur "defined but not used"
    (void)SVC_UUID;
    (void)CHR_TEMP;

    ble_hs_id_infer_auto(0, &g_own_addr_type);
    struct ble_gap_disc_params d_params = {0};
    d_params.passive = 0;
    d_params.filter_duplicates = 1;
    ble_gap_disc(g_own_addr_type, BLE_HS_FOREVER, &d_params, gap_event_cb, NULL);
    ESP_LOGI(TAG, "Scan démarré...");
}

void ble_host_task(void *param) {
    nimble_port_run();
}

void app_main(void) {
    nvs_flash_init();
    init_uart();
    nimble_port_init();
    ble_svc_gap_device_name_set("S3_GATEWAY");
    ble_hs_cfg.sync_cb = on_sync;
    nimble_port_freertos_init(ble_host_task);
}