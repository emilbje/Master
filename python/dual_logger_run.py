import csv
import json
import re
import threading
import time
import webbrowser
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import urlparse

import serial


# ==========================================
# INNSTILLINGER
# ==========================================
MOTOR_PORT = "COM7"
IMU_PORT = "COM10"
BAUDRATE = 115200

WEB_HOST = "127.0.0.1"
WEB_PORT = 8766
AUTO_OPEN_BROWSER = True

STEPS_PER_MM = 50.0
MOTOR_SPEED_STEPS_PER_SEC = 50
MAX_FORCE_LIMIT_KG = 90.0

CONTACT_THRESHOLD_KG = 0.01
RECOVERY_ZERO_THRESHOLD_KG = 0.02
RECOVERY_EXTRA_UP_MM = 2.0
RECOVERY_TIMEOUT_SEC = 60.0

# Sikkerhetsstopp hvis testen kjores langt ned uten aa naa stoppkriteriet.
MAX_TEST_TRAVEL_MM = 120.0

MOTOR_SPEED_CMD = f"S{MOTOR_SPEED_STEPS_PER_SEC}"
MOTOR_FORCE_LIMIT_CMD = f"F{MAX_FORCE_LIMIT_KG:g}"
MOTOR_SET_ZERO_CMD = "P"
MOTOR_STOP_CMD = "Z"
MOTOR_UP_CONTINUOUS_CMD = "C1"
MOTOR_DOWN_CONTINUOUS_CMD = "C-1"
MOTOR_STATUS_CMD = "R"
MOTOR_TARE_CMD = "T"
MOTOR_STREAM_ON_CMD = "STREAMON"

IMU_STREAM_ON_CMD = "s"
IMU_STREAM_OFF_CMD = "x"
# ==========================================


motor_pattern = re.compile(
    r"(?:DATA|RESULTAT|STATUS)\s+Pos:\s*([-+]?\d+(?:\.\d+)?)\s*mm\s+Kraft:\s*([-+]?\d+(?:\.\d+)?)\s*kg",
    re.IGNORECASE,
)

latest_motor = {
    "pos_mm": None,
    "force_kg": None,
    "raw": None,
    "result_seq": 0,
    "status_seq": 0,
}

latest_imu = {
    "imu0": None,
    "imu1": None,
    "imu2": None,
    "imu3": None,
    "imu4": None,
    "imu5": None,
    "imu6": None,
    "imu7": None,
    "raw": None,
}

state_lock = threading.RLock()
state_condition = threading.Condition(state_lock)
serial_write_lock = threading.Lock()
shutdown_event = threading.Event()
test_done_event = threading.Event()
abort_event = threading.Event()

log_lock = threading.Lock()
log_lines = deque(maxlen=600)

action_lock = threading.Lock()
action_thread = None
current_action = None
jog_active = False
jog_direction = None

motor_serial = None
imu_serial = None
active_test = None
last_csv_file = None


@dataclass
class TestSession:
    mode: str
    target_value: float
    csv_file: object
    writer: csv.DictWriter
    filename: str
    start_pos_mm: Optional[float] = None
    contact_detected: bool = False
    contact_mm: Optional[float] = None
    stop_requested: bool = False
    stop_reason: Optional[str] = None
    started_at: float = field(default_factory=time.monotonic)


def log(message):
    timestamp = datetime.now().strftime("%H:%M:%S")
    line = f"[{timestamp}] {message}"
    print(line, flush=True)
    with log_lock:
        log_lines.append(line)


def get_logs():
    with log_lock:
        return list(log_lines)


def create_csv(mode, target_value):
    target_text = f"{target_value:.2f}".rstrip("0").rstrip(".").replace(".", "p")
    filename = f"downpress_{mode}_{target_text}_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
    csv_path = Path(filename)
    csv_file = csv_path.open("w", newline="", encoding="utf-8")

    fieldnames = [
        "force_kg",
        "rel_defl_mm",
        "imu0",
        "imu1",
        "imu2",
        "imu3",
        "imu4",
        "imu5",
        "imu6",
        "imu7",
    ]

    writer = csv.DictWriter(csv_file, fieldnames=fieldnames, delimiter=";")
    writer.writeheader()
    return csv_file, writer, str(csv_path)


def send_cmd(ser, cmd, echo=True):
    if ser is None:
        return False

    with serial_write_lock:
        ser.write((cmd + "\n").encode("utf-8"))
        ser.flush()

    if echo:
        log(f"SEND {cmd}")
    return True


def parse_motor_line(line):
    match = motor_pattern.search(line)
    if not match:
        return None
    pos_mm = float(match.group(1))
    force_kg = float(match.group(2))
    return pos_mm, force_kg


def parse_imu_line(line):
    if not line:
        return None

    parts = line.strip().split("\t")
    if len(parts) != 10:
        return None

    if parts[0].upper() != "IMU":
        return None

    try:
        return {
            "imu0": float(parts[2]),
            "imu1": float(parts[3]),
            "imu2": float(parts[4]),
            "imu3": float(parts[5]),
            "imu4": float(parts[6]),
            "imu5": float(parts[7]),
            "imu6": float(parts[8]),
            "imu7": float(parts[9]),
        }
    except ValueError:
        return None


def get_latest_motor_snapshot():
    with state_lock:
        return latest_motor["pos_mm"], latest_motor["force_kg"], latest_motor["raw"]


def get_result_seq():
    with state_lock:
        return latest_motor["result_seq"]


def get_status_seq():
    with state_lock:
        return latest_motor["status_seq"]


def wait_for_result_after(previous_seq, timeout_sec):
    end_time = time.monotonic() + timeout_sec
    with state_condition:
        while latest_motor["result_seq"] <= previous_seq and not shutdown_event.is_set():
            remaining = end_time - time.monotonic()
            if remaining <= 0:
                return False
            state_condition.wait(remaining)
        return latest_motor["result_seq"] > previous_seq


def wait_for_status_after(previous_seq, timeout_sec):
    end_time = time.monotonic() + timeout_sec
    with state_condition:
        while latest_motor["status_seq"] <= previous_seq and not shutdown_event.is_set():
            remaining = end_time - time.monotonic()
            if remaining <= 0:
                return False
            state_condition.wait(remaining)
        return latest_motor["status_seq"] > previous_seq


def finite_move_timeout_sec(mm):
    mm_per_sec = MOTOR_SPEED_STEPS_PER_SEC / STEPS_PER_MM
    return max(8.0, abs(mm) / mm_per_sec + 8.0)


def status_text(prefix):
    pos_mm, force_kg, _ = get_latest_motor_snapshot()
    pos_text = "ukjent" if pos_mm is None else f"{pos_mm:.3f} mm"
    force_text = "ukjent" if force_kg is None else f"{force_kg:.3f} kg"
    return f"{prefix} Pos: {pos_text} | Kraft: {force_text}"


def request_motor_status(motor_ser):
    previous_seq = get_status_seq()
    send_cmd(motor_ser, MOTOR_STATUS_CMD)
    wait_for_status_after(previous_seq, 2.0)
    log(status_text("STATUS"))


def update_active_test_from_motor(pos_mm, force_kg, line_label):
    global active_test

    command_stop = False
    session_to_report = None

    with state_lock:
        session = active_test
        if session is None:
            return False

        if session.start_pos_mm is None:
            session.start_pos_mm = pos_mm

        if not session.contact_detected and force_kg >= CONTACT_THRESHOLD_KG:
            session.contact_detected = True
            session.contact_mm = pos_mm
            log(f"KONTAKT Nullpunkt for nedpress ved posisjon {pos_mm:.3f} mm")

        rel_defl_mm = None
        if session.contact_detected and session.contact_mm is not None:
            rel_defl_mm = pos_mm - session.contact_mm

        row = {
            "force_kg": force_kg,
            "rel_defl_mm": rel_defl_mm,
            "imu0": latest_imu["imu0"],
            "imu1": latest_imu["imu1"],
            "imu2": latest_imu["imu2"],
            "imu3": latest_imu["imu3"],
            "imu4": latest_imu["imu4"],
            "imu5": latest_imu["imu5"],
            "imu6": latest_imu["imu6"],
            "imu7": latest_imu["imu7"],
        }
        session.writer.writerow(row)
        session.csv_file.flush()

        should_stop = False
        stop_reason = None

        if session.mode == "mm":
            if rel_defl_mm is not None and rel_defl_mm <= -session.target_value:
                should_stop = True
                stop_reason = f"nedpress {abs(rel_defl_mm):.3f} mm"

        elif session.mode == "kg":
            if force_kg >= session.target_value:
                should_stop = True
                stop_reason = f"kraft {force_kg:.3f} kg"

        if session.start_pos_mm is not None:
            total_down_mm = session.start_pos_mm - pos_mm
            if total_down_mm >= MAX_TEST_TRAVEL_MM:
                should_stop = True
                stop_reason = f"sikkerhetsstopp {total_down_mm:.1f} mm total nedkjoring"

        if should_stop and not session.stop_requested:
            session.stop_requested = True
            session.stop_reason = stop_reason
            session_to_report = session
            test_done_event.set()
            command_stop = line_label != "RESULTAT"

    if session_to_report is not None:
        log(f"STOPP {session_to_report.stop_reason}")

    return command_stop


def motor_reader_thread(motor_ser):
    while not shutdown_event.is_set():
        try:
            line = motor_ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            upper = line.upper()
            if not upper.startswith("DATA"):
                log(f"MOTOR {line}")

            parsed = parse_motor_line(line)
            if parsed is None:
                continue

            pos_mm, force_kg = parsed
            line_parts = upper.split()
            line_label = line_parts[0] if line_parts else ""

            with state_condition:
                latest_motor["pos_mm"] = pos_mm
                latest_motor["force_kg"] = force_kg
                latest_motor["raw"] = line
                latest_motor["status_seq"] += 1
                if line_label == "RESULTAT":
                    latest_motor["result_seq"] += 1
                state_condition.notify_all()

            if update_active_test_from_motor(pos_mm, force_kg, line_label):
                try:
                    send_cmd(motor_ser, MOTOR_STOP_CMD)
                except Exception:
                    pass

        except Exception as exc:
            if not shutdown_event.is_set():
                log(f"FEIL motor_reader_thread {exc}")
                shutdown_event.set()


def imu_reader_thread(imu_ser):
    while not shutdown_event.is_set():
        try:
            line = imu_ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            parsed = parse_imu_line(line)
            if parsed is None:
                if not line.upper().startswith("IMU"):
                    log(f"IMU {line}")
                continue

            with state_lock:
                latest_imu.update(parsed)
                latest_imu["raw"] = line

        except Exception as exc:
            if not shutdown_event.is_set():
                log(f"FEIL imu_reader_thread {exc}")
                shutdown_event.set()


def perform_finite_move(motor_ser, mm):
    previous_result_seq = get_result_seq()
    send_cmd(motor_ser, f"G{mm:.3f}")
    timeout_sec = finite_move_timeout_sec(mm)
    deadline = time.monotonic() + timeout_sec

    log(f"MOVE Venter paa flytt {mm:.3f} mm")
    while not abort_event.is_set():
        if wait_for_result_after(previous_result_seq, 0.2):
            log(status_text("MOVE FERDIG"))
            return True
        if time.monotonic() > deadline:
            break

    if abort_event.is_set():
        log("MOVE Avbrutt. Stopper motor.")
        send_cmd(motor_ser, MOTOR_STOP_CMD)
        wait_for_result_after(previous_result_seq, 5.0)
        return False

    log(f"ADVARSEL Fikk ikke RESULTAT innen {timeout_sec:.1f} s")
    return False


def finite_move_action(motor_ser, mm):
    return perform_finite_move(motor_ser, mm)


def recover_after_test(motor_ser):
    log("RETUR Starter automatisk retur uten logging")
    send_cmd(motor_ser, MOTOR_FORCE_LIMIT_CMD)
    time.sleep(0.2)

    _, force_kg, _ = get_latest_motor_snapshot()
    if force_kg is None:
        request_motor_status(motor_ser)
        _, force_kg, _ = get_latest_motor_snapshot()

    reached_zero = force_kg is not None and force_kg <= RECOVERY_ZERO_THRESHOLD_KG

    if reached_zero:
        log("RETUR Lastcelle er allerede naer 0 kg")
    else:
        previous_result_seq = get_result_seq()
        send_cmd(motor_ser, MOTOR_UP_CONTINUOUS_CMD)
        start_time = time.monotonic()
        last_print = 0.0

        while time.monotonic() - start_time < RECOVERY_TIMEOUT_SEC and not abort_event.is_set():
            time.sleep(0.1)
            _, force_kg, _ = get_latest_motor_snapshot()

            if force_kg is not None and force_kg <= RECOVERY_ZERO_THRESHOLD_KG:
                log(f"RETUR Kraft {force_kg:.3f} kg. Stopper oppkjoring")
                send_cmd(motor_ser, MOTOR_STOP_CMD)
                reached_zero = True
                break

            now = time.monotonic()
            if now - last_print >= 1.0:
                log(status_text("RETUR"))
                last_print = now

        if abort_event.is_set():
            log("RETUR Avbrutt fra interface")
            send_cmd(motor_ser, MOTOR_STOP_CMD)
            wait_for_result_after(previous_result_seq, 5.0)
            return False

        if not reached_zero:
            log("ADVARSEL Retur stoppet paa timeout")
            send_cmd(motor_ser, MOTOR_STOP_CMD)
            wait_for_result_after(previous_result_seq, 5.0)
            return False

        wait_for_result_after(previous_result_seq, 10.0)

    log(f"RETUR Kjorer {RECOVERY_EXTRA_UP_MM:.1f} mm ekstra opp")
    return finite_move_action(motor_ser, RECOVERY_EXTRA_UP_MM)


def close_active_test():
    global active_test, last_csv_file

    with state_lock:
        session = active_test
        active_test = None

    if session is not None:
        session.csv_file.close()
        last_csv_file = session.filename
        log(f"CSV Lagret i {session.filename}")
        if session.stop_reason:
            log(f"TEST Stoppgrunn: {session.stop_reason}")


def run_logged_test(motor_ser, imu_ser, mode, target_value):
    global active_test

    if mode == "kg":
        send_cmd(motor_ser, f"F{target_value:.3f}")
    else:
        send_cmd(motor_ser, MOTOR_FORCE_LIMIT_CMD)
    time.sleep(0.2)

    previous_status_seq = get_status_seq()
    send_cmd(motor_ser, MOTOR_SET_ZERO_CMD)
    wait_for_status_after(previous_status_seq, 2.0)

    if imu_ser is not None:
        send_cmd(imu_ser, IMU_STREAM_ON_CMD)
        time.sleep(0.5)

    csv_file, writer, filename = create_csv(mode, target_value)
    session = TestSession(
        mode=mode,
        target_value=target_value,
        csv_file=csv_file,
        writer=writer,
        filename=filename,
    )

    with state_lock:
        session.start_pos_mm = latest_motor["pos_mm"]
        active_test = session
        test_done_event.clear()

    unit = "mm etter kontakt" if mode == "mm" else "kg"
    log(f"TEST Starter nedpress til {target_value:.3f} {unit}. Logger til {filename}")

    previous_result_seq = get_result_seq()
    send_cmd(motor_ser, MOTOR_DOWN_CONTINUOUS_CMD)

    aborted = False
    last_print = 0.0

    while not test_done_event.wait(0.1):
        if abort_event.is_set():
            aborted = True
            with state_lock:
                if active_test is not None:
                    active_test.stop_requested = True
                    active_test.stop_reason = "avbrutt fra interface"
            log("AVBRUTT Stopper motoren")
            send_cmd(motor_ser, MOTOR_STOP_CMD)
            break

        now = time.monotonic()
        if now - last_print >= 1.0:
            log(status_text("TEST"))
            last_print = now

    wait_for_result_after(previous_result_seq, 10.0)

    if imu_ser is not None:
        send_cmd(imu_ser, IMU_STREAM_OFF_CMD)

    close_active_test()
    send_cmd(motor_ser, MOTOR_FORCE_LIMIT_CMD)

    if aborted:
        log("TEST Retur hoppet over fordi testen ble avbrutt")
        return

    recover_after_test(motor_ser)


def tare_action(motor_ser):
    previous_status_seq = get_status_seq()
    send_cmd(motor_ser, MOTOR_TARE_CMD)
    wait_for_status_after(previous_status_seq, 5.0)
    log(status_text("TARE"))


def set_zero_action(motor_ser):
    previous_status_seq = get_status_seq()
    send_cmd(motor_ser, MOTOR_SET_ZERO_CMD)
    wait_for_status_after(previous_status_seq, 2.0)
    log(status_text("NULLPUNKT"))


def set_speed_action(motor_ser, steps_per_sec):
    global MOTOR_SPEED_STEPS_PER_SEC, MOTOR_SPEED_CMD

    steps_per_sec = int(steps_per_sec)
    if steps_per_sec < 1:
        steps_per_sec = 1
    if steps_per_sec > 20000:
        steps_per_sec = 20000

    MOTOR_SPEED_STEPS_PER_SEC = steps_per_sec
    MOTOR_SPEED_CMD = f"S{steps_per_sec}"
    send_cmd(motor_ser, MOTOR_SPEED_CMD)
    log(f"HASTIGHET Satt til {steps_per_sec} steps/s")


def get_action_label():
    with action_lock:
        thread_alive = action_thread is not None and action_thread.is_alive()
        if thread_alive:
            return current_action
        if jog_active:
            return f"Jogg {jog_direction}"
    return None


def action_runner(label, target, args):
    global current_action

    log(f"ACTION Starter: {label}")
    try:
        target(*args)
    except Exception as exc:
        log(f"FEIL i handling '{label}': {exc}")
    finally:
        with action_lock:
            current_action = None
        abort_event.clear()
        log(f"ACTION Ferdig: {label}")


def start_action(label, target, *args):
    global action_thread, current_action

    with action_lock:
        if action_thread is not None and action_thread.is_alive():
            return False, f"Opptatt: {current_action}"
        if jog_active:
            return False, "Stopp jogg forst"

        abort_event.clear()
        current_action = label
        action_thread = threading.Thread(
            target=action_runner,
            args=(label, target, args),
            daemon=True,
        )
        action_thread.start()

    return True, "Startet"


def start_jog(direction):
    global jog_active, jog_direction

    with action_lock:
        if action_thread is not None and action_thread.is_alive():
            return False, f"Opptatt: {current_action}"
        if jog_active:
            return False, "Jogg er allerede aktiv"

        cmd = MOTOR_UP_CONTINUOUS_CMD if direction == "opp" else MOTOR_DOWN_CONTINUOUS_CMD
        if not send_cmd(motor_serial, cmd):
            return False, "Motorport ikke klar"

        jog_active = True
        jog_direction = direction

    log(f"JOGG Starter {direction}")
    return True, "Jogg startet"


def stop_all():
    global jog_active, jog_direction

    abort_event.set()
    test_done_event.set()

    with state_lock:
        if active_test is not None and not active_test.stop_requested:
            active_test.stop_requested = True
            active_test.stop_reason = "stoppet fra interface"

    try:
        send_cmd(motor_serial, MOTOR_STOP_CMD)
    except Exception:
        pass

    with action_lock:
        jog_active = False
        jog_direction = None

    log("STOPP Sendte stoppkommando")


def send_raw_command(cmd):
    cmd = str(cmd).strip()
    if not cmd:
        return False, "Tom kommando"
    if len(cmd) > 80:
        return False, "Kommandoen er for lang"
    send_cmd(motor_serial, cmd)
    return True, "Sendt"


def get_status_payload():
    with state_lock:
        motor = {
            "pos_mm": latest_motor["pos_mm"],
            "force_kg": latest_motor["force_kg"],
            "raw": latest_motor["raw"],
        }
        imu = dict(latest_imu)
        test = None
        if active_test is not None:
            test = {
                "mode": active_test.mode,
                "target_value": active_test.target_value,
                "filename": active_test.filename,
                "contact_detected": active_test.contact_detected,
                "contact_mm": active_test.contact_mm,
                "stop_reason": active_test.stop_reason,
            }

    return {
        "motor": motor,
        "imu": imu,
        "active_test": test,
        "action": get_action_label(),
        "jog_active": jog_active,
        "jog_direction": jog_direction,
        "last_csv_file": last_csv_file,
        "logs": get_logs(),
        "settings": {
            "motor_port": MOTOR_PORT,
            "imu_port": IMU_PORT,
            "speed_steps_per_sec": MOTOR_SPEED_STEPS_PER_SEC,
            "max_force_limit_kg": MAX_FORCE_LIMIT_KG,
            "recovery_zero_threshold_kg": RECOVERY_ZERO_THRESHOLD_KG,
            "recovery_extra_up_mm": RECOVERY_EXTRA_UP_MM,
        },
    }


HTML_PAGE = r"""<!doctype html>
<html lang="no">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Dual Logger Kontroll</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f4f5f2;
      --panel: #ffffff;
      --ink: #1f2423;
      --muted: #5c6662;
      --line: #d7ddd8;
      --green: #247a4b;
      --green-2: #e6f3eb;
      --orange: #a45a13;
      --orange-2: #fff0de;
      --red: #b3261e;
      --red-2: #fde8e5;
      --blue: #2f5d9b;
      --blue-2: #e7eef8;
      --code: #161a1a;
    }

    * { box-sizing: border-box; }

    body {
      margin: 0;
      font-family: Arial, Helvetica, sans-serif;
      background: var(--bg);
      color: var(--ink);
    }

    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      padding: 16px 22px;
      border-bottom: 1px solid var(--line);
      background: var(--panel);
      position: sticky;
      top: 0;
      z-index: 10;
    }

    h1 {
      margin: 0;
      font-size: 22px;
      font-weight: 700;
      letter-spacing: 0;
    }

    h2 {
      margin: 0 0 12px;
      font-size: 16px;
      letter-spacing: 0;
    }

    main {
      display: grid;
      grid-template-columns: minmax(320px, 440px) minmax(360px, 1fr);
      gap: 16px;
      padding: 16px;
      max-width: 1380px;
      margin: 0 auto;
    }

    section {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 16px;
    }

    .left,
    .right {
      display: grid;
      gap: 16px;
      align-content: start;
    }

    .status-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 10px;
    }

    .metric {
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 10px;
      min-height: 76px;
      display: grid;
      align-content: center;
      gap: 4px;
    }

    .metric span {
      color: var(--muted);
      font-size: 12px;
    }

    .metric strong {
      font-size: 24px;
      white-space: nowrap;
    }

    .row {
      display: flex;
      gap: 8px;
      align-items: end;
      flex-wrap: wrap;
    }

    .field {
      display: grid;
      gap: 5px;
      min-width: 120px;
      flex: 1 1 120px;
    }

    label {
      font-size: 12px;
      color: var(--muted);
    }

    input {
      width: 100%;
      height: 38px;
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 0 10px;
      font-size: 15px;
      background: #fff;
      color: var(--ink);
    }

    button {
      height: 38px;
      border: 1px solid transparent;
      border-radius: 6px;
      padding: 0 12px;
      font-size: 14px;
      font-weight: 700;
      cursor: pointer;
      color: var(--ink);
      background: #eef1ee;
      white-space: nowrap;
    }

    button:hover { filter: brightness(0.97); }
    button:disabled { opacity: 0.45; cursor: not-allowed; }

    .btn-green { background: var(--green-2); border-color: #b9dec7; color: var(--green); }
    .btn-orange { background: var(--orange-2); border-color: #f0c99f; color: var(--orange); }
    .btn-red { background: var(--red-2); border-color: #efb6b0; color: var(--red); }
    .btn-blue { background: var(--blue-2); border-color: #b9ccea; color: var(--blue); }
    .btn-wide { flex: 1 1 140px; }

    .pill {
      display: inline-flex;
      align-items: center;
      min-height: 30px;
      border: 1px solid var(--line);
      border-radius: 999px;
      padding: 4px 12px;
      color: var(--muted);
      background: #fff;
      max-width: 52vw;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }

    .pill.busy {
      color: var(--orange);
      background: var(--orange-2);
      border-color: #f0c99f;
    }

    .log {
      height: min(54vh, 560px);
      min-height: 320px;
      overflow: auto;
      background: var(--code);
      color: #e9eeee;
      border-radius: 6px;
      padding: 12px;
      font-family: Consolas, "Courier New", monospace;
      font-size: 13px;
      line-height: 1.45;
      white-space: pre-wrap;
    }

    .muted {
      color: var(--muted);
      font-size: 13px;
      overflow-wrap: anywhere;
    }

    .split {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
    }

    .raw-row {
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 8px;
    }

    @media (max-width: 900px) {
      main { grid-template-columns: 1fr; }
      header { align-items: flex-start; flex-direction: column; }
      .status-grid { grid-template-columns: 1fr; }
      .split { grid-template-columns: 1fr; }
      .pill { max-width: 100%; }
    }
  </style>
</head>
<body>
  <header>
    <h1>Dual Logger Kontroll</h1>
    <div id="actionPill" class="pill">Klar</div>
  </header>

  <main>
    <div class="left">
      <section>
        <h2>Status</h2>
        <div class="status-grid">
          <div class="metric">
            <span>Posisjon</span>
            <strong id="posValue">-</strong>
          </div>
          <div class="metric">
            <span>Kraft</span>
            <strong id="forceValue">-</strong>
          </div>
          <div class="metric">
            <span>Hastighet</span>
            <strong id="speedValue">-</strong>
          </div>
        </div>
        <p id="lastCsv" class="muted"></p>
      </section>

      <section>
        <h2>Kj&oslash;ring</h2>
        <div class="row">
          <div class="field">
            <label for="moveMm">Millimeter</label>
            <input id="moveMm" type="number" min="0.001" step="0.1" value="2">
          </div>
          <button class="btn-green btn-wide" onclick="moveMm('up')">&uarr; Opp</button>
          <button class="btn-orange btn-wide" onclick="moveMm('down')">&darr; Ned</button>
        </div>
        <div class="row" style="margin-top: 10px;">
          <button class="btn-green btn-wide" onclick="startJog('opp')">&uarr; Jogg opp</button>
          <button class="btn-orange btn-wide" onclick="startJog('ned')">&darr; Jogg ned</button>
          <button class="btn-red btn-wide" onclick="stopAll()">&#9632; Stopp</button>
        </div>
      </section>

      <section>
        <h2>Test</h2>
        <div class="split">
          <div class="field">
            <label for="targetMm">Nedpress mm</label>
            <input id="targetMm" type="number" min="0.001" step="0.1" value="25">
          </div>
          <div class="field">
            <label for="targetKg">Stopp kg</label>
            <input id="targetKg" type="number" min="0.01" step="0.1" value="10">
          </div>
        </div>
        <div class="row" style="margin-top: 10px;">
          <button class="btn-blue btn-wide" onclick="startTest('mm')">Start mm-test</button>
          <button class="btn-blue btn-wide" onclick="startTest('kg')">Start kg-test</button>
          <button class="btn-red btn-wide" onclick="stopAll()">&#9632; Avbryt</button>
        </div>
      </section>

      <section>
        <h2>Oppsett</h2>
        <div class="row">
          <button onclick="tare()">Tare</button>
          <button onclick="zeroMotor()">Nullstill posisjon</button>
          <button onclick="readStatus()">Les status</button>
        </div>
        <div class="row" style="margin-top: 10px;">
          <div class="field">
            <label for="speedInput">Steps/s</label>
            <input id="speedInput" type="number" min="1" max="20000" step="1" value="50">
          </div>
          <button onclick="setSpeed()">Sett hastighet</button>
        </div>
      </section>
    </div>

    <div class="right">
      <section>
        <h2>Logg</h2>
        <div id="logBox" class="log"></div>
      </section>

      <section>
        <h2>Serial</h2>
        <div class="raw-row">
          <input id="rawCommand" type="text" placeholder="R, G2, G-2, C1, C-1, Z">
          <button class="btn-blue" onclick="sendRaw()">Send</button>
        </div>
        <p id="rawMotor" class="muted"></p>
      </section>
    </div>
  </main>

  <script>
    let isBusy = false;
    let lastLogText = "";

    function numberOrNull(id) {
      const value = Number(document.getElementById(id).value);
      return Number.isFinite(value) ? value : null;
    }

    async function postJson(path, payload = {}) {
      const response = await fetch(path, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload)
      });
      const data = await response.json();
      if (!response.ok || data.ok === false) {
        throw new Error(data.error || data.message || "Ukjent feil");
      }
      return data;
    }

    async function moveMm(direction) {
      const mm = numberOrNull("moveMm");
      if (!mm || mm <= 0) return;
      await postJson("/api/move", { direction, mm }).catch(alert);
      await refresh();
    }

    async function startJog(direction) {
      await postJson("/api/jog", { direction }).catch(alert);
      await refresh();
    }

    async function startTest(mode) {
      const target = mode === "mm" ? numberOrNull("targetMm") : numberOrNull("targetKg");
      if (!target || target <= 0) return;
      await postJson("/api/test", { mode, target }).catch(alert);
      await refresh();
    }

    async function stopAll() {
      await postJson("/api/stop").catch(alert);
      await refresh();
    }

    async function tare() {
      await postJson("/api/tare").catch(alert);
      await refresh();
    }

    async function zeroMotor() {
      await postJson("/api/zero").catch(alert);
      await refresh();
    }

    async function readStatus() {
      await postJson("/api/status-command").catch(alert);
      await refresh();
    }

    async function setSpeed() {
      const stepsPerSec = numberOrNull("speedInput");
      if (!stepsPerSec || stepsPerSec <= 0) return;
      await postJson("/api/speed", { stepsPerSec }).catch(alert);
      await refresh();
    }

    async function sendRaw() {
      const input = document.getElementById("rawCommand");
      const command = input.value.trim();
      if (!command) return;
      await postJson("/api/raw", { command }).then(() => { input.value = ""; }).catch(alert);
      await refresh();
    }

    function setButtonState(action, jogActive) {
      isBusy = Boolean(action || jogActive);
      document.querySelectorAll("button").forEach((button) => {
        const stopButton = button.textContent.includes("Stopp") || button.textContent.includes("Avbryt");
        button.disabled = isBusy && !stopButton;
      });
    }

    function renderStatus(data) {
      const pos = data.motor.pos_mm;
      const force = data.motor.force_kg;
      document.getElementById("posValue").textContent = pos === null ? "-" : `${pos.toFixed(3)} mm`;
      document.getElementById("forceValue").textContent = force === null ? "-" : `${force.toFixed(3)} kg`;
      document.getElementById("speedValue").textContent = `${data.settings.speed_steps_per_sec}`;
      const speedInput = document.getElementById("speedInput");
      if (document.activeElement !== speedInput) {
        speedInput.value = data.settings.speed_steps_per_sec;
      }

      const pill = document.getElementById("actionPill");
      const label = data.action || (data.jog_active ? `Jogg ${data.jog_direction}` : "Klar");
      pill.textContent = label;
      pill.classList.toggle("busy", Boolean(data.action || data.jog_active));

      document.getElementById("lastCsv").textContent = data.last_csv_file ? `Siste CSV: ${data.last_csv_file}` : "";
      document.getElementById("rawMotor").textContent = data.motor.raw ? `Siste motorlinje: ${data.motor.raw}` : "";
      setButtonState(data.action, data.jog_active);
    }

    function renderLogs(logs) {
      const box = document.getElementById("logBox");
      const text = logs.join("\n");
      if (text === lastLogText) return;
      const shouldStick = box.scrollHeight - box.scrollTop - box.clientHeight < 32;
      box.textContent = text;
      if (shouldStick) box.scrollTop = box.scrollHeight;
      lastLogText = text;
    }

    async function refresh() {
      const response = await fetch("/api/status");
      const data = await response.json();
      renderStatus(data);
      renderLogs(data.logs || []);
    }

    refresh();
    setInterval(refresh, 300);
  </script>
</body>
</html>
"""


class RequestHandler(BaseHTTPRequestHandler):
    server_version = "DualLoggerHTTP/1.0"

    def log_message(self, _format, *_args):
        return

    def _read_json(self):
        length = int(self.headers.get("Content-Length", "0") or "0")
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        if not raw:
            return {}
        return json.loads(raw.decode("utf-8"))

    def _send_bytes(self, payload, content_type, status=HTTPStatus.OK):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def _send_json(self, payload, status=HTTPStatus.OK):
        data = json.dumps(payload).encode("utf-8")
        self._send_bytes(data, "application/json; charset=utf-8", status)

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self._send_bytes(HTML_PAGE.encode("utf-8"), "text/html; charset=utf-8")
            return
        if parsed.path == "/api/status":
            self._send_json(get_status_payload())
            return
        self._send_json({"ok": False, "error": "Ikke funnet"}, HTTPStatus.NOT_FOUND)

    def do_POST(self):
        parsed = urlparse(self.path)

        try:
            payload = self._read_json()

            if parsed.path == "/api/move":
                direction = payload.get("direction")
                mm = float(payload.get("mm", 0))
                if direction not in {"up", "down"} or mm <= 0:
                    raise ValueError("Ugyldig retning eller mm")
                signed_mm = mm if direction == "up" else -mm
                label = f"Kjorer {'opp' if direction == 'up' else 'ned'} {mm:.3f} mm"
                ok, message = start_action(label, finite_move_action, motor_serial, signed_mm)
                self._send_json({"ok": ok, "message": message}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
                return

            if parsed.path == "/api/jog":
                direction = payload.get("direction")
                if direction not in {"opp", "ned"}:
                    raise ValueError("Ugyldig jogg-retning")
                ok, message = start_jog(direction)
                self._send_json({"ok": ok, "message": message}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
                return

            if parsed.path == "/api/test":
                mode = payload.get("mode")
                target = float(payload.get("target", 0))
                if mode not in {"mm", "kg"} or target <= 0:
                    raise ValueError("Ugyldig test")
                if mode == "kg" and target > MAX_FORCE_LIMIT_KG:
                    raise ValueError(f"Maks kraft er {MAX_FORCE_LIMIT_KG:g} kg")
                label = f"Test {target:.3f} {'mm' if mode == 'mm' else 'kg'}"
                ok, message = start_action(label, run_logged_test, motor_serial, imu_serial, mode, target)
                self._send_json({"ok": ok, "message": message}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
                return

            if parsed.path == "/api/stop":
                stop_all()
                self._send_json({"ok": True})
                return

            if parsed.path == "/api/tare":
                ok, message = start_action("Tare lastcelle", tare_action, motor_serial)
                self._send_json({"ok": ok, "message": message}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
                return

            if parsed.path == "/api/zero":
                ok, message = start_action("Nullstill posisjon", set_zero_action, motor_serial)
                self._send_json({"ok": ok, "message": message}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
                return

            if parsed.path == "/api/status-command":
                ok, message = start_action("Les status", request_motor_status, motor_serial)
                self._send_json({"ok": ok, "message": message}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
                return

            if parsed.path == "/api/speed":
                steps_per_sec = int(payload.get("stepsPerSec", 0))
                ok, message = start_action("Sett hastighet", set_speed_action, motor_serial, steps_per_sec)
                self._send_json({"ok": ok, "message": message}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
                return

            if parsed.path == "/api/raw":
                command = payload.get("command", "")
                ok, message = send_raw_command(command)
                self._send_json({"ok": ok, "message": message}, HTTPStatus.OK if ok else HTTPStatus.BAD_REQUEST)
                return

            self._send_json({"ok": False, "error": "Ikke funnet"}, HTTPStatus.NOT_FOUND)

        except Exception as exc:
            self._send_json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)


def open_serial_ports():
    log("OPPSTART Aapner motorport")
    motor_ser = serial.Serial(MOTOR_PORT, BAUDRATE, timeout=0.2)

    imu_ser = None
    try:
        log("OPPSTART Aapner IMU-port")
        imu_ser = serial.Serial(IMU_PORT, BAUDRATE, timeout=0.2)
    except Exception as exc:
        log(f"ADVARSEL Kunne ikke aapne IMU-port {IMU_PORT}: {exc}")
        log("ADVARSEL Programmet kan styre motoren, men IMU-felt blir tomme")

    time.sleep(2.0)
    motor_ser.reset_input_buffer()
    if imu_ser is not None:
        imu_ser.reset_input_buffer()

    threading.Thread(target=motor_reader_thread, args=(motor_ser,), daemon=True).start()
    if imu_ser is not None:
        threading.Thread(target=imu_reader_thread, args=(imu_ser,), daemon=True).start()

    send_cmd(motor_ser, MOTOR_SPEED_CMD)
    time.sleep(0.2)
    send_cmd(motor_ser, MOTOR_FORCE_LIMIT_CMD)
    time.sleep(0.2)
    send_cmd(motor_ser, MOTOR_STREAM_ON_CMD)
    time.sleep(0.2)
    request_motor_status(motor_ser)

    return motor_ser, imu_ser


def close_serial_ports(motor_ser, imu_ser):
    shutdown_event.set()

    try:
        send_cmd(motor_ser, MOTOR_STOP_CMD)
    except Exception:
        pass

    try:
        send_cmd(imu_ser, IMU_STREAM_OFF_CMD)
    except Exception:
        pass

    time.sleep(0.2)

    try:
        motor_ser.close()
    except Exception:
        pass

    try:
        if imu_ser is not None:
            imu_ser.close()
    except Exception:
        pass


def run_web_server():
    server = ThreadingHTTPServer((WEB_HOST, WEB_PORT), RequestHandler)
    url = f"http://{WEB_HOST}:{WEB_PORT}"
    log(f"WEB Interface klart: {url}")

    if AUTO_OPEN_BROWSER:
        threading.Timer(0.4, lambda: webbrowser.open(url)).start()

    try:
        server.serve_forever()
    finally:
        server.server_close()


def main():
    global motor_serial, imu_serial

    try:
        motor_serial, imu_serial = open_serial_ports()
        run_web_server()
    except KeyboardInterrupt:
        log("AVSLUTTET Ctrl+C")
    except Exception as exc:
        log(f"FEIL {exc}")
    finally:
        stop_all()
        close_active_test()
        if motor_serial is not None:
            close_serial_ports(motor_serial, imu_serial)
        log("FERDIG")


if __name__ == "__main__":
    main()
