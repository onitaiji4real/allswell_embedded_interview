"""Setup check: connects to SITL and prints live telemetry.

Run this first. If you see altitude/battery/mode updating, you're good to go.
"""

import sys
import time

from pymavlink import mavutil

CONNECTION_STRING = "tcp:127.0.0.1:5760"


def main() -> None:
    print(f"Connecting to {CONNECTION_STRING} ...")
    conn = mavutil.mavlink_connection(CONNECTION_STRING)

    hb = conn.wait_heartbeat(timeout=30)
    if hb is None:
        print("No heartbeat received. Is the simulator running? (docker compose up -d)")
        sys.exit(1)
    print(f"Heartbeat from system {conn.target_system} component {conn.target_component}")

    # Ask the autopilot to stream telemetry at 4 Hz.
    conn.mav.request_data_stream_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4,  # Hz
        1,  # start
    )

    mode = "?"
    armed = False
    last_print = 0.0

    while True:
        msg = conn.recv_match(blocking=True, timeout=5)
        if msg is None:
            print("Telemetry stream stalled.")
            sys.exit(1)

        mtype = msg.get_type()
        if mtype == "HEARTBEAT":
            mode = mavutil.mode_string_v10(msg)
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        elif mtype == "GLOBAL_POSITION_INT" and time.time() - last_print > 1.0:
            last_print = time.time()
            print(
                f"mode={mode:<10} armed={armed!s:<5} "
                f"lat={msg.lat / 1e7:.6f} lon={msg.lon / 1e7:.6f} "
                f"alt_rel={msg.relative_alt / 1000:.1f}m"
            )
        elif mtype == "SYS_STATUS" and msg.voltage_battery != 0xFFFF:
            # Printed less prominently; battery matters for Part 3.
            pass


if __name__ == "__main__":
    main()
