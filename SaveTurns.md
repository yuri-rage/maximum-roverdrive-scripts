# SaveTurns

SaveTurns (`SaveTurns.lua`) records a path while you drive it manually. It saves the vehicle's position as a mission waypoint at each turn, so the path can be replayed in AUTO.

Requires `modules/roverdrive_util.lua`. See [Installation](#installation).

> **CAUTION:** This script modifies the stored mission. Use it at your own risk.

## Parameters

Parameters appear after the script runs for the first time. Reboot after installing, then set them. Changes to `STRN_RC_OPT` require a reboot.

| Parameter | Default | Units | Description |
|---|---|---|---|
| `STRN_RC_OPT` | 303 | | RC aux option (300-307) assigned to the 3-position switch (`RCn_OPTION`) |
| `STRN_MIN_DIST` | 2.0 | m | Minimum distance between saved waypoints |
| `STRN_HDG_DELTA` | 8.0 | deg | Save a waypoint after a heading change of this size |
| `STRN_VERBOSE` | 1 | | GCS message level: 0 = errors only, 1 = normal |

The defaults match the previous version's values.

## Usage

| Switch position | Action |
|---|---|
| LOW | Standby |
| MIDDLE | Record turns |
| HIGH | Clear the mission |

**Recording (MIDDLE):**
- When recording starts, the current position is saved as the first waypoint.
- Once the vehicle is more than `STRN_MIN_DIST` from the last saved waypoint, a new waypoint is saved whenever the heading differs by more than `STRN_HDG_DELTA` from the heading at which it left that waypoint.
- Waypoints are appended to the current mission. If the mission is empty, a home entry is created first.

**Clearing (HIGH):**
- Clears the mission, leaving only the home position.
- Nothing more happens until the switch moves.
- Moving the switch directly to MIDDLE starts a new recording.

**AUTO mode:** recording stops when the vehicle enters AUTO, and the switch is ignored while in AUTO. Leaving AUTO with the switch at MIDDLE starts a new recording session.

**Home position:** creating or clearing a mission requires home to be set. If it isn't, the script warns and does nothing until the switch moves.

**Switch travel:** going from HIGH to LOW passes through MIDDLE. If the script reads the switch during that pass (it checks every 200 ms), it starts recording and saves a waypoint at the current position.

## Messages

| Message | Meaning |
|---|---|
| `Awaiting AHRS initialization...` / `Ready` | Script loaded / ready to record |
| `Recording turns` / `Recording stopped` | Recording session started / ended |
| `WP n saved` | Waypoint saved at mission index n |
| `WP n - failed to save` | Mission write failed |
| `Mission cleared` | Mission cleared (HIGH) |
| `Home not set - cannot create mission` | Home position required |
| `AUTO mode - recording canceled` | Vehicle entered AUTO while recording |
| `RC option n not found` | No RC channel has `STRN_RC_OPT` assigned; the script stops |
| `Internal error: ...` | Script error; the script restarts after 1 s |

## Installation

1. Copy `SaveTurns.lua` to `APM/scripts/` and `modules/roverdrive_util.lua` to `APM/scripts/modules/`.
2. Assign a 3-position switch: set `RCn_OPTION` to `STRN_RC_OPT` (default 303).
3. Reboot. The parameter table key is 60, which must be unique among scripts on the vehicle.
