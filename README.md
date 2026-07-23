# Allswell Technology Services - Embedded Software Engineer

Welcome! In this task you'll write Python code that commands a simulated
quadcopter over MAVLink — the same protocol our real aircraft speak. The
simulator is ArduPilot SITL (the same autopilot firmware that flies our
drones, compiled for your desktop), so everything you write here maps
directly onto real flight code.

**We'd rather see two parts done well than three done** sloppily. If you run out of time, describe what you'd do next in your notes.

---

## Setup (15 minutes)

Requirements: Docker, Python 3.10+. Works on macOS (Intel & Apple Silicon),
Linux, and Windows.

```bash
# 1. Start the simulator (first build compiles ArduPilot — give it ~10 min)
docker compose up -d --build

# 2. Install Python deps
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# 3. Verify: you should see live telemetry within ~20 seconds
python starter/telemetry.py
```

The simulated drone sits at a field near Canberra, Australia, and listens
for MAVLink on `tcp:127.0.0.1:5760`. To reset the drone to a clean state at
any time: `docker compose restart`.

Tip: you can also connect a ground station (e.g. QGroundControl, or Mission
Planner if you're on Windows) to `tcp:127.0.0.1:5762` to watch your code fly
the drone on a map. Not required, but very helpful for debugging.

---



## Rules

- Use `pymavlink` (provided). Please don't use higher-level SDKs
(DroneKit, MAVSDK) — we want to see how you handle the protocol itself.
- Standard library only beyond that (plus `pytest` if you write tests).
- **No blind** `time.sleep()` **as a substitute for confirmation.** Commands can
be rejected and messages can be missed; your code should confirm outcomes
by observing telemetry and acknowledgements, with sensible timeouts.
- Extend `starter/drone.py` or restructure it however you like — the
skeleton is a suggestion, not a requirement.

Useful references:

- MAVLink messages: [https://mavlink.io/en/messages/common.html](https://mavlink.io/en/messages/common.html)
- ArduCopter flight modes: GUIDED = accepts position commands from a
companion computer; RTL = return to launch.
- Commands are sent with `COMMAND_LONG` and confirmed with `COMMAND_ACK`.

---



## Part 1 — Arm and take off

Write `part1_takeoff.py`:

1. Connect to the drone and wait until it is ready to fly. A guided
  takeoff needs a position estimate, which takes **~40–60 s after boot**
   while the simulated GPS and EKF converge (hint: `EKF_STATUS_REPORT`
   flags, or `STATUSTEXT` messages). Arming too early gets rejected — and
   an armed drone that doesn't take off auto-disarms after ~10 s.
2. Switch to GUIDED mode. Confirm the mode change actually happened.
3. Arm the motors. Confirm via acknowledgement **and** telemetry.
4. Command a takeoff to **15 m** above home.
5. Print altitude as it climbs, and print `REACHED` when within 0.5 m of
  target. Exit cleanly.

Every step must handle failure (rejected command, timeout) by printing a
clear error and exiting non-zero — not by hanging or crashing with a
traceback.

## Part 2 — Fly a square

Write `part2_square.py` (may reuse Part 1 for takeoff):

1. Take off to 15 m.
2. Fly a square pattern, **80 m per side**, returning to the start corner
  (hint: `SET_POSITION_TARGET_GLOBAL_INT`, or compute corner coordinates
   and send goto commands).
3. Consider a corner "reached" when within **2 m horizontally**. Print
  progress (distance to next corner) at ~1 Hz while flying.
4. After the last corner, command RTL and wait until the drone has landed
  and disarmed. Then exit.



## Part 3 — Battery failsafe monitor

Real missions get aborted by real failures. Write `part3_failsafe.py`:

1. Start the same square mission from Part 2.
2. **Concurrently**, monitor battery voltage (`SYS_STATUS`). If voltage
  drops below **11.0 V**, immediately abort the mission: command RTL,
   log the abort reason with a timestamp, and stop sending goto commands.
3. To make this testable, your script should also *cause* the failure:
  ~20 seconds into the mission, set the simulator parameter
   `SIM_BATT_VOLTAGE` to `10.5` using the MAVLink parameter protocol, and
   confirm the parameter was actually set.
4. The expected output: mission starts → voltage drops → your monitor
  catches it mid-square → drone returns and lands → clean exit with a log
   that tells the story.

The interesting part here is the design: mission logic and safety
monitoring have to run at the same time and share one MAVLink connection.
Threads, asyncio, or a single-loop state machine are all acceptable —
tell us why you picked yours.

## Bonus (only if you have time left)

Pick one, at most:

- Upload the square as a proper AUTO mission using the MAVLink mission
protocol and fly it in AUTO mode.
- A geofence monitor: define a circle around home, warn on approach,
force RTL on breach.

---



## Deliverables

1. Your code (`part1_takeoff.py`, `part2_square.py`, `part3_failsafe.py`,
  plus any shared modules) along with any other code used to ensure proper functionality.
2. `NOTES.md` — half a page: design decisions, what you'd do differently
  with more time, anything that surprised you.

Send the whole folder back as a git repo link.# allswell_embedded_interview
