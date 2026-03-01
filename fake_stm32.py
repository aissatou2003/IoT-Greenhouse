import re
import sys
import time
import serial

def extract_seq(s: str):
    m = re.search(r"SEQ=(\d+)", s)
    return int(m.group(1)) if m else None

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM8"
    baud = 115200

    print(f"Fake STM32 sur {port} @ {baud}")
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,          # non-bloquant
            write_timeout=0.2
        )
    except Exception as e:
        print(f"ERREUR ouverture {port}: {e}")
        sys.exit(1)

    # Petit flush
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    buf = b""

    while True:
        try:
            chunk = ser.read(256)
            if chunk:
                buf += chunk

                # On traite ligne par ligne (délimiteur \n)
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip(b"\r").strip()

                    if not line:
                        continue

                    try:
                        txt = line.decode(errors="replace")
                    except Exception:
                        txt = str(line)

                    print(f"Reçu brut : {txt!r}")

                    seq = extract_seq(txt)
                    if seq is None:
                        reply = "ACK;SEQ=?\n"
                    else:
                        reply = f"ACK;SEQ={seq}\n"

                    ser.write(reply.encode())
                    ser.flush()
                    print(f"Envoyé : {reply.strip()}")

            else:
                # petit sleep pour éviter de bouffer CPU
                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\nStop.")
            break
        except Exception as e:
            print(f"ERREUR runtime: {e}")
            time.sleep(0.5)

    ser.close()

if __name__ == "__main__":
    main()
