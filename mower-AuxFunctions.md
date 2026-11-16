# Mower Aux Functions

Mower Aux Functions (`mower-AuxFunctions.lua`) controls a mower's PTO (blade drive) relay and its parking brake.

Requires `modules/roverdrive_util.lua`. See [Installation](#installation).

> **CAUTION:** This script controls the PTO and parking brake. Use it at your own risk.

## Parameters

Parameters appear after the script runs for the first time. Reboot after installing, then set them. Changes to all except `MOW_BRK_DELAY` require a reboot.

| Parameter | Default | Units | Description |
|---|---|---|---|
| `MOW_PTO_RC_OPT` | 301 | | RC aux option (300-307) assigned to the PTO switch (`RCn_OPTION`) |
| `MOW_PTO_RELAY` | 3 | | PTO relay number (1 = RELAY1). `RELAYn_FUNCTION` should be 1 (Relay) |
| `MOW_BRK_SRV_FN` | 95 | | Servo output function for the parking brake (94-109 = Script1-16) |
| `MOW_BRK_DELAY` | 2.0 | s | Delay before the parking brake engages |

### Migrating from the previous version

The previous version called `relay:on(2)`, which uses 0-indexed relay numbers, so it drove RELAY3. `MOW_PTO_RELAY` is 1-indexed and defaults to 3, which is the same relay. The other defaults match the old script's values.

## PTO

| Event | PTO |
|---|---|
| PTO switch raised while armed | Engages |
| PTO switch lowered | Disengages |
| Disarm | Disengages |
| Crash detected | Disengages |
| Script internal error | Disengages |
| Arming with the PTO switch already raised | Stays off |

The PTO engages only when the switch moves. After a forced disengage, or when arming with the switch already up, lower and raise the switch to engage. The relay is commanded off when the script starts.

`relay:on()`/`relay:off()` only drive relays whose `RELAYn_FUNCTION` is 1 (Relay). If the PTO relay is configured otherwise, the script reports an error at startup and on every engage attempt. The PTO won't operate, but the parking brake still works.

## Parking brake

The brake output is driven to its maximum (engaged) or minimum (released) through the `MOW_BRK_SRV_FN` servo function.

- **Engages** `MOW_BRK_DELAY` seconds after disarming or entering HOLD. If the vehicle leaves HOLD or arms in a driving mode before the delay ends, the brake is not applied.
- **Releases** immediately when the vehicle is armed in any mode other than HOLD.

Crash detection does not engage the brake directly, because the wheels may still need to be driven to recover, for example when stuck in mud. Rover's crash failsafe switches to HOLD, so the brake engages after the delay. It releases as soon as a driving mode such as MANUAL is selected.

Selecting a driving mode makes throttle available at the same moment the brake starts releasing. Depending on how fast the brake actuator moves, the wheels may briefly push against a brake that is still releasing.

## Messages

| Message | Meaning |
|---|---|
| `Parking brake/PTO active` | Script loaded |
| `PTO engaged` / `PTO disengaged` | PTO state changed by the switch |
| `PTO disengaged - disarmed` / `PTO disengaged - crash` | PTO forced off |
| `Parking brake on` / `Parking brake off` | Brake state changed |
| `RELAYn_FUNCTION is not 1 (Relay) - PTO will not operate` | PTO relay misconfigured |
| `Internal error: ...` | Script error; PTO turned off and the script restarts after 1 s |

## Installation

1. Copy `mower-AuxFunctions.lua` to `APM/scripts/` and `modules/roverdrive_util.lua` to `APM/scripts/modules/`.
2. Assign the PTO switch: set `RCn_OPTION` to `MOW_PTO_RC_OPT` (default 301).
3. Set `RELAYn_FUNCTION = 1` for the PTO relay.
4. Assign the brake output: set `SERVOn_FUNCTION` to `MOW_BRK_SRV_FN` (default 95). Set that channel's `SERVOn_MIN`/`SERVOn_MAX` so that maximum engages the brake.
5. Reboot. The parameter table key is 61, which must be unique among scripts on the vehicle.

The script stops at startup if no RC channel has the PTO option assigned, or no servo has the brake function.

When used with `crash-actions.lua`, a `CRASH_ACTx = 2` (relay off) action on the PTO relay is redundant with this script but harmless.
