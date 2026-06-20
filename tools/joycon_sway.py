#!/usr/bin/env python3
"""
Joy-Con sagittal sway controller for Optimus biped.

Both Joy-Cons connected; left stick vertical on the Left Joy-Con controls
hip forward/backward offset. Joy-Con is read on a background thread so the
main send loop never blocks on HID polling.

Usage:
    pip install -r requirements.txt
    python joycon_sway.py --host <ESP32_IP> [--port 81] [--amp 20]
"""
import argparse
import json
import sys
import threading
import time

try:
    from pyjoycon import JoyCon, get_L_id, get_R_id
except ImportError:
    sys.exit("ERROR: pyjoycon not installed. Run: pip install -r requirements.txt")

try:
    import websocket
except ImportError:
    sys.exit("ERROR: websocket-client not installed. Run: pip install -r requirements.txt")

STICK_MIN = 1200   # calibrated: actual min at full backward deflection
STICK_MAX = 3318   # calibrated: actual max at full forward deflection
STICK_MID = 2173   # calibrated from observed center value at rest
DEADZONE  = 0.05


def stick_to_mm(raw, amp):
    if raw >= STICK_MID:
        norm = (raw - STICK_MID) / (STICK_MAX - STICK_MID)   # 0..+1
    else:
        norm = (raw - STICK_MID) / (STICK_MID - STICK_MIN)   # -1..0
    if abs(norm) < DEADZONE:
        norm = 0.0
    return round(norm * amp, 1)


# Shared state written by reader thread, read by send loop
_latest_mm   = 0.0
_latest_lock = threading.Lock()


def joycon_reader(joycon, amp):
    """Background thread — polls Joy-Con and updates _latest_mm."""
    global _latest_mm
    while True:
        try:
            status = joycon.get_status()
            raw    = status["analog-sticks"]["left"]["vertical"]
            val    = stick_to_mm(raw, amp)
            with _latest_lock:
                _latest_mm = val
            print(f"[stick] raw={raw}  norm={((raw-STICK_MID)/STICK_MID):+.3f}  mm={val:+.1f}", flush=True)
        except Exception as e:
            print(f"[reader] Joy-Con read error: {e}")
        time.sleep(0.005)   # 200 Hz read rate — HID latency is the real limit


def main():
    parser = argparse.ArgumentParser(description="Joy-Con → Optimus sway controller")
    parser.add_argument("--host", required=True,          help="ESP32 IP shown on serial monitor")
    parser.add_argument("--port", type=int, default=81,   help="WebSocket port (default: 81)")
    parser.add_argument("--amp",  type=float, default=20.0, help="Sway amplitude mm (default: 20)")
    parser.add_argument("--hz",   type=float, default=50.0, help="Send rate Hz (default: 50)")
    args = parser.parse_args()

    # Connect both Joy-Cons (required when using both simultaneously)
    print("[joycon_sway] Looking for Joy-Cons ...")
    try:
        l_id = get_L_id()
        r_id = get_R_id()
        joycon_l = JoyCon(*l_id)
        joycon_r = JoyCon(*r_id)
        print(f"[joycon_sway] Left Joy-Con:  {l_id}")
        print(f"[joycon_sway] Right Joy-Con: {r_id}")
    except Exception as e:
        sys.exit(f"ERROR: Could not find Joy-Cons: {e}")

    # Start background reader on Left Joy-Con left stick
    t = threading.Thread(target=joycon_reader, args=(joycon_l, args.amp), daemon=True)
    t.start()
    print("[joycon_sway] Reader thread started.")

    # Connect WebSocket
    url = f"ws://{args.host}:{args.port}"
    print(f"[joycon_sway] Connecting to {url} ...")
    ws = websocket.WebSocket()
    try:
        ws.connect(url)
    except Exception as e:
        sys.exit(f"ERROR: WebSocket connect failed: {e}")
    print("[joycon_sway] Connected. Move left stick to control sway. Ctrl-C to stop.")

    interval = 1.0 / args.hz
    try:
        while True:
            t0 = time.monotonic()

            with _latest_lock:
                target = _latest_mm

            print(f"[sway] target={target:+.1f}mm  raw stick → {target}", flush=True)
            try:
                ws.send(json.dumps({"cmd": "set_sway_target", "value": target}))
            except Exception as e:
                print(f"[joycon_sway] Send error: {e} — reconnecting ...")
                try:
                    ws.connect(url)
                except Exception:
                    pass

            elapsed = time.monotonic() - t0
            sleep   = interval - elapsed
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        print("\n[joycon_sway] Stopping — sending neutral.")
        try:
            ws.send(json.dumps({"cmd": "set_sway_target", "value": 0.0}))
        except Exception:
            pass
        ws.close()


if __name__ == "__main__":
    main()
