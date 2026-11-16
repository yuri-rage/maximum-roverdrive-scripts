# AutoSupervisor

AutoSupervisor (`rover-AutoSupervisor.lua`) watches Rover AUTO missions and adjusts them for GPS quality and ground conditions. It replaces the former `rover-MinFixType.lua` and `rover-TerrainDetector.lua` scripts and adds a third function:

1. **GPS fix monitor:** pauses the mission when the GPS fix type stays below a threshold, and resumes it once a good fix has persisted.
2. **Terrain detector:** detects rough terrain from IMU data and slows the vehicle, then resumes normal speed once the terrain smooths out.
3. **Mission speed restore:** restores a `DO_CHANGE_SPEED` mission speed that Rover drops when AUTO is re-entered.

Requires `modules/roverdrive_util.lua`. See [Installation](#installation).

> **CAUTION:** This script can change flight modes and vehicle speed during autonomous operation. Use it at your own risk.

## Parameters

Parameters appear after the script runs for the first time. Reboot after installing, then set them. Unless noted, changes take effect immediately.

| Parameter | Default | Units | Description |
|---|---|---|---|
| `ASV_VERBOSE` | 2 | | GCS message level: 0 = errors only, 1 = normal, 2 = debug |
| `ASV_FIX_GPS` | 1 | | GPS instance to monitor (1 = GPS1) |
| `ASV_FIX_MIN` | 6 | | Minimum acceptable fix type (see [Fix types](#fix-types)). 0 disables the GPS fix monitor |
| `ASV_FIX_BAD_T` | 1.6 | s | A below-minimum fix must persist this long before the mission pauses |
| `ASV_FIX_GOOD_T` | 0.6 | s | An acceptable fix must persist this long before the mission resumes |
| `ASV_FIX_PAUSE` | 0 | mode | Mode used to pause the mission (0 = MANUAL) |
| `ASV_FIX_THRMODE` | 4 | mode | Mode used to pause the mission if the throttle stick is not centered (4 = HOLD) |
| `ASV_TRN_SPD` | 0 | m/s | Rough terrain speed. 0, or a value above the mission speed, means half the mission speed. -1 disables the terrain detector |
| `ASV_TRN_GZ` | 1.33 | G | Slow down when the vertical acceleration impulse exceeds this |
| `ASV_TRN_GYRO` | 28 | deg/s | Slow down when the roll or pitch rate exceeds this |
| `ASV_TRN_TIME` | 7.5 | s | Remain slowed for at least this long after the last rough terrain detection |
| `ASV_TRN_GZ_GAIN` | 0.9 | | Multiplier on `ASV_TRN_GZ` while slowed. Values below 1 make it harder to resume normal speed |
| `ASV_TRN_GY_GAIN` | 0.8 | | Multiplier on `ASV_TRN_GYRO` while slowed. Values below 1 make it harder to resume normal speed |

The script also reads these vehicle parameters at startup (reboot after changing them): `MODE_CH`, `MODE1`-`MODE6`, `RCMAP_THROTTLE`, and that channel's `RCn_TRIM` and `RCn_DZ`. It reads `WP_SPEED` continuously.

### Migrating from the previous scripts

| Old setting | New parameter |
|---|---|
| `SCR_USER1` (TerrainDetector rough speed) | `ASV_TRN_SPD` |
| `GPS_INSTANCE = 0` (0-indexed) | `ASV_FIX_GPS = 1` (1-indexed) |
| `MIN_FIX_TYPE`, `BAD_FIX_TIMEOUT`, `GOOD_FIX_TIMEOUT` (ms) | `ASV_FIX_MIN`, `ASV_FIX_BAD_T`, `ASV_FIX_GOOD_T` (s) |
| `MSN_PAUSE_MODE`, `THR_SAFEGUARD_MODE` | `ASV_FIX_PAUSE`, `ASV_FIX_THRMODE` |
| `IMPULSE_GZ_THRESHOLD`, `GYRO_RATE_THRESHOLD`, `ROUGH_TERRAIN_TIMEOUT_MS` (ms) | `ASV_TRN_GZ`, `ASV_TRN_GYRO`, `ASV_TRN_TIME` (s) |
| `IMPULSE_GAIN`, `GYRO_GAIN` | `ASV_TRN_GZ_GAIN`, `ASV_TRN_GY_GAIN` |
| `VERBOSE_MODE` | `ASV_VERBOSE` |

The defaults match the old scripts' values, so `ASV_TRN_SPD` is the only parameter that needs setting during migration. Remove both old scripts from the autopilot.

## GPS fix monitor

While a mission is running, the script checks the fix type of GPS `ASV_FIX_GPS`.

- **Pause:** if the fix stays below `ASV_FIX_MIN` for `ASV_FIX_BAD_T` seconds, the script switches to `ASV_FIX_PAUSE`. If the throttle stick is outside its deadzone at that moment, it switches to `ASV_FIX_THRMODE` instead, so that a stick that isn't centered doesn't drive the vehicle in MANUAL. Once the stick is centered, it moves from `ASV_FIX_THRMODE` to `ASV_FIX_PAUSE`.
- **Resume:** if the fix stays at or above `ASV_FIX_MIN` for `ASV_FIX_GOOD_T` seconds, the script switches back to AUTO and the mission continues from where it stopped.
- **Debounce:** the timers require the condition to hold continuously. A single good sample restarts the pause timer, and a single bad sample restarts the resume timer.

While paused, the script cancels the pause and stops trying to resume if:
- the RC mode switch leaves its AUTO position, or the mode channel signal is lost;
- the vehicle is disarmed;
- the mode changes to anything other than the pause mode, the throttle mode or AUTO, for example from a GCS or a failsafe.

If something switches back to AUTO while the fix is still inadequate (for example a GCS command), the script reverts to the pause mode.

The RC mode switch must have an AUTO position (`MODEn = 10`) for a pause to resume. The script uses it to tell a GPS pause apart from a deliberate mode change.

### Fix types

| Value | Fix type |
|---|---|
| 0 | No GPS |
| 1 | No fix |
| 2 | 2D fix |
| 3 | 3D fix |
| 4 | DGPS |
| 5 | RTK Float |
| 6 | RTK Fixed |
| 7 | Static fixed |
| 8 | PPP, 3D |

With a moving baseline GPS pair, GPS1 is typically the base (`GPS1_TYPE = 17`). The script monitors whichever instance `ASV_FIX_GPS` selects.

## Terrain detector

While a mission is running, the script samples the IMU every 25 ms.

- **Vertical impulse:** the change in vertical acceleration (in G) between consecutive samples. A sharp jolt registers as a large impulse.
- **Roll/pitch rate:** the larger of the absolute roll and pitch rates (deg/s).

When either value exceeds its threshold (`ASV_TRN_GZ` or `ASV_TRN_GYRO`), the vehicle slows to the rough terrain speed. While slowed, the thresholds are multiplied by `ASV_TRN_GZ_GAIN` and `ASV_TRN_GY_GAIN`, so smaller bumps keep it slowed. Each detection restarts the `ASV_TRN_TIME` timer. When the timer runs out, the vehicle resumes normal speed.

**Rough terrain speed:**
- `ASV_TRN_SPD` if it is between 0 and the mission speed;
- half the mission speed if `ASV_TRN_SPD` is 0;
- half the mission speed if `ASV_TRN_SPD` is above the mission speed, with a warning once per mission run;
- terrain detection is disabled if `ASV_TRN_SPD` is -1 (or any negative value).

**Normal (mission) speed** is the speed of the latest `DO_CHANGE_SPEED` the mission has executed, or `WP_SPEED` if there is none.

**When speed is commanded.** The script commands a speed only to slow down or to resume normal speed. It sends nothing at mission start or during smooth running. Two special cases:
- If a `DO_CHANGE_SPEED` runs while the vehicle is slowed, the rough terrain speed is applied again. Otherwise the mission's speed change would end the slowdown early.
- If rough terrain was detected within `ASV_TRN_TIME` before a GPS pause, the vehicle is slowed again after resuming, because re-entering AUTO resets the speed.

**Pivot turns.** Speed changes wait until the vehicle is moving at least half the rough terrain speed. This avoids changing speed during pivot turns.

### Tuning

1. Set `ASV_VERBOSE = 2` and run a mission over typical terrain.
2. Debug messages report each detection and its value:
   - `Impulse - 1.42 Gz` (vertical impulse)
   - `Transient - 30.80 deg/s` (roll/pitch rate)
3. When the vehicle resumes normal speed, `Max - x.xx Gz ... y.yy deg/s` reports the largest impulse and rate seen while it was slowed.
4. If smooth ground triggers slowdowns, raise the thresholds. If real rough patches are missed, lower them.

The old TerrainDetector's `Max` message reported absolute vertical acceleration, so its values aren't comparable with the impulse values reported here.

## Mission speed restore

When AUTO is re-entered (resuming after a GPS pause, or switching out of AUTO and back), Rover resets the speed to `WP_SPEED`. It re-runs only the do-commands that follow the current waypoint. That means a `DO_CHANGE_SPEED` from an earlier leg is lost.

The script checks the mission after each re-entry. If the latest `DO_CHANGE_SPEED` comes before the current waypoint, the script restores it once the vehicle is moving. No speed is commanded in these cases:
- a fresh mission start;
- missions without `DO_CHANGE_SPEED`;
- when the firmware re-runs the speed change itself;
- while the terrain detector is controlling the speed. It returns to the mission speed when the slowdown ends.

This also works when the terrain detector is disabled.

## Messages

Messages are prefixed with `ASV: `.

| Message | Level | Meaning |
|---|---|---|
| `Script active` | normal | Script loaded |
| `GPSn - <fix type>` | debug | Fix dropped below the minimum; pause timer started |
| `Mission paused - <fix type>` | normal | Bad fix persisted for `ASV_FIX_BAD_T` |
| `Throttle not neutral!` | normal | Paused in `ASV_FIX_THRMODE` because the throttle stick was not centered |
| `Throttle neutral` | normal | Throttle centered; switching to `ASV_FIX_PAUSE` |
| `Cannot resume - <fix type>` | normal | AUTO was selected while the fix is inadequate; reverted to the pause mode |
| `Mission resumed` | normal | Good fix persisted for `ASV_FIX_GOOD_T` |
| `Pause canceled - <reason>` | normal | Mode switch moved, disarmed, or mode changed |
| `Slowing for rough terrain` | normal | Rough terrain speed applied |
| `Resuming normal speed` | normal | Mission speed applied |
| `Impulse - x.xx Gz` / `Transient - x.xx deg/s` | debug | Rough terrain detection and its value |
| `Max - x.xx Gz ... y.yy deg/s` | debug | Largest values while slowed |
| `ASV_TRN_SPD > mission spd - using half` | normal | Invalid rough speed setting |
| `Mission speed restored - x.xx m/s` | normal | Lost `DO_CHANGE_SPEED` restored after AUTO re-entry |
| `Internal error: ...` | always | Script error; the script restarts after 1 s |

## Installation

1. Copy `rover-AutoSupervisor.lua` to `APM/scripts/` and `modules/roverdrive_util.lua` to `APM/scripts/modules/`.
2. Make sure scripting is enabled (`SCR_ENABLE = 1`) and reboot.
3. Set the `ASV_` parameters. The parameter table key is 62, which must be unique among scripts on the vehicle.

Credit to @ktrussell for the terrain detection concept, first presented in [this live stream](https://www.youtube.com/watch?v=UdXGXjigxAo&t=7155s).
