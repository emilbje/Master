import csv
import time
import threading
import serial
from serial.tools import list_ports

BAUD = 115200
OUTFILE = "imu_log.csv"
N_IMU = 8
HEADER = ["defl_mm"] + [f"imu{i}" for i in range(N_IMU)]
WRITE_DELIM = ";"  # bra for norsk Excel

def pick_port():
    ports = list(list_ports.comports())
    if not ports:
        raise RuntimeError("Fant ingen serial-porter.")
    print("Tilgjengelige porter:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  -  {p.description}")
    idx = int(input("Velg port-indeks: ").strip())
    return ports[idx].device

def ensure_header(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            has = f.read(1) != ""
    except FileNotFoundError:
        has = False

    if not has:
        with open(path, "a", newline="", encoding="utf-8") as f:
            csv.writer(f, delimiter=WRITE_DELIM).writerow(HEADER)

def parse_line(line: str):
    line = line.strip()
    if not line:
        return None

    # støtt både tab og ;
    parts = line.split("\t") if "\t" in line else line.split(";")
    if len(parts) != 1 + N_IMU:
        return None

    try:
        return [float(p.replace(",", ".")) for p in parts]
    except ValueError:
        return None

def read_one_snapshot(ser: serial.Serial, timeout_s=2.0):
    """
    Leser linjer til vi får en som ser ut som data (riktig antall felt).
    Returnerer rå linje (str) og parsed (liste float), eller (None, None).
    """
    t_end = time.time() + timeout_s
    while time.time() < t_end:
        raw = ser.readline().decode("utf-8", errors="replace")
        parsed = parse_line(raw)
        if parsed is not None:
            return raw.strip(), parsed
    return None, None

def main():
    port = pick_port()
    ser = serial.Serial(port, BAUD, timeout=0.2)
    time.sleep(2.0)  # la boardet restarte etter port-open

    # Tøm buffer for boot-tekster
    ser.reset_input_buffer()

    ensure_header(OUTFILE)

    print("\nKlar.")
    print("Enter = send 'p' og logg én linje")
    print("Skriv 'p' + Enter = samme")
    print("Skriv 'q' + Enter = avslutt\n")

    with open(OUTFILE, "a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f, delimiter=WRITE_DELIM)

        while True:
            cmd = input("> ").strip().lower()

            if cmd == "q":
                break

            if cmd == "" or cmd == "p":
                # Send trigger
                ser.write(b"p")
                ser.flush()

                raw, parsed = read_one_snapshot(ser, timeout_s=2.0)
                if raw is None:
                    print("Ingen gyldig linje mottatt (timeout). Sjekk at Arduino faktisk printer ved 'p'.")
                    continue

                # Vis hva vi fikk
                print("RECV:", raw)

                # Append til CSV
                writer.writerow(parsed)
                f.flush()
                print(f"APPENDED -> {OUTFILE}")

            else:
                print("Ukjent kommando. Bruk Enter/p/q.")

    ser.close()
    print("Ferdig.")

if __name__ == "__main__":
    main()