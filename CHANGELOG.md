# Changelog

Notable changes to these scripts, newest first. The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## 2026-11-16

All scripts except `crash-actions.lua` now require `modules/roverdrive_util.lua`, installed in `APM/scripts/modules/`. User settings moved from script variables to parameters; the defaults match the previous values. See each script's documentation for migration details.

### Added

- `rover-AutoSupervisor.lua`, which combines `rover-MinFixType.lua` and `rover-TerrainDetector.lua` (`ASV_` parameters).
- AutoSupervisor mission speed restore: a `DO_CHANGE_SPEED` from an earlier leg, which Rover drops when AUTO is re-entered (for example after a GPS pause), is restored.
- `modules/roverdrive_util.lua`: shared GCS messaging with verbosity levels, parameter table setup, and a state machine runner that reports errors and restarts the script instead of stopping it.
- Parameters for all user settings: `ASV_` (key 62), `MOW_` (key 61), `STRN_` (key 60).
- Documentation for each script: `rover-AutoSupervisor.md`, `mower-AuxFunctions.md`, `SaveTurns.md`, `crash-actions.md`.
- `.luarc.json` for Lua language server type checking against a sibling ArduPilot checkout.

### Changed

- **AutoSupervisor (GPS):**
  - A below-minimum fix must persist for the full `ASV_FIX_BAD_T` before the mission pauses. Previously a single check after the timeout decided.
  - Resuming likewise requires an acceptable fix for the full `ASV_FIX_GOOD_T`.
  - During a pause, a mode change to anything other than AUTO or the pause modes cancels the pause instead of being overridden. This includes a GCS or failsafe mode change.
  - Losing the RC mode channel signal also cancels the pause.
- **AutoSupervisor (terrain):**
  - The rough terrain speed is set by `ASV_TRN_SPD` instead of `SCR_USER1`.
  - After rough terrain, the vehicle resumes the mission's speed (the latest executed `DO_CHANGE_SPEED`) instead of always `WP_SPEED`.
  - A `DO_CHANGE_SPEED` executed while slowed doesn't end the slowdown.
  - If rough terrain was detected shortly before a GPS pause, the vehicle is slowed again after resuming.
  - The debug `Max` message reports the peak vertical impulse instead of absolute vertical acceleration.
- **mower-AuxFunctions:**
  - The PTO turns off on disarm, on crash detection, and on a script error. Lower and raise the switch to re-engage.
  - The PTO relay is commanded off at startup.
  - The parking brake delay no longer pauses PTO switch handling, and it restarts if the vehicle leaves HOLD and re-enters it.
  - `MOW_PTO_RELAY` is 1-indexed. The default of 3 matches the previous `relay:on(2)`.
  - A PTO relay whose `RELAYn_FUNCTION` isn't 1 now gets an error message; the brake still works.
- **SaveTurns:** creating or clearing a mission requires home to be set.
- **crash-actions:**
  - Relays must have `RELAYn_FUNCTION = 1`.
  - Servos must use a function that `DO_SET_SERVO` accepts.
  - Invalid instances are reported when the action runs.
- GCS messages are prefixed with the script name. Some wording changed.

### Fixed

- **TerrainDetector:** if the slowdown timer expired during a pivot turn, the vehicle was slowed again for another `ASV_TRN_TIME` instead of resuming normal speed.
- **MinFixType:** an unknown fix type stopped the script.
- **SaveTurns:**
  - The first waypoint wasn't saved when recording started at a heading of about -81° (279°).
  - A missing AHRS location stopped the script.
  - Moving from clear (HIGH) straight to record (MIDDLE) reused the previous session's waypoint state.
  - Recording into an empty mission no longer writes a waypoint into the home slot.
  - The home altitude is now converted from cm to m.
- **crash-actions:**
  - A rejected `DO_SET_SERVO` was reported as success.
  - Servo channels with `SERVOn_FUNCTION = 0`, which `DO_SET_SERVO` drives, were rejected as unconfigured.
  - Relays and servos that the firmware ignores (wrong function) failed silently.
- No script stops permanently on a runtime error anymore (`pcall` protection).

### Removed

- `rover-MinFixType.lua` and `rover-TerrainDetector.lua`, replaced by `rover-AutoSupervisor.lua`. Remove them from the autopilot.

## 2026-10-24

- Initial import of the scripts running on the vehicle: `SaveTurns.lua`, `crash-actions.lua`, `mower-AuxFunctions.lua`, `rover-MinFixType.lua` and `rover-TerrainDetector.lua`.
