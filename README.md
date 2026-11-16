# Maximum Roverdrive Scripts

I run these scripts on my ArduPilot Rover-based zero turn lawnmower.

This repository is mostly for my personal use and version control. Use its contents at your own risk.

> [!NOTE]
> These scripts are my own work, and most have counterparts in the official ArduPilot repository (the terrain detection concept is credited to @ktrussell). The versions here have been modified, some heavily, with AI coding assistance (Claude Code), so they may differ substantially from the upstream scripts of the same name.

## Scripts

| Script | Purpose | Param prefix (table key) |
|---|---|---|
| `rover-AutoSupervisor.lua` | Pauses AUTO missions on poor GPS fix; slows for rough terrain (replaces `rover-MinFixType.lua` and `rover-TerrainDetector.lua`) | `ASV_` (62) |
| `mower-AuxFunctions.lua` | Parking brake and PTO relay control | `MOW_` (61) |
| `SaveTurns.lua` | Records turns as mission waypoints via an RC switch | `STRN_` (60) |
| `crash-actions.lua` | Configurable post-crash actions; self-contained, based on the upstream applet with additional relay/servo validity checks | `CRASH_` (188) |

Each script has its own documentation covering parameters, behavior and messages: [`rover-AutoSupervisor.md`](rover-AutoSupervisor.md), [`mower-AuxFunctions.md`](mower-AuxFunctions.md), [`SaveTurns.md`](SaveTurns.md) and [`crash-actions.md`](crash-actions.md).

## Installation

- Copy the scripts to `APM/scripts/` on the autopilot.
- Copy `modules/roverdrive_util.lua` to `APM/scripts/modules/`. All scripts except `crash-actions.lua` require it.
- Parameter table keys must be unique among all scripts on the vehicle.

## Migrating from the previous versions

- `rover-TerrainDetector.lua` read the rough terrain speed from `SCR_USER1`; set `ASV_TRN_SPD` instead.
- `mower-AuxFunctions.lua` previously called `relay:on(2)` (0-indexed, i.e. RELAY3). `MOW_PTO_RELAY` is 1-indexed and defaults to 3.
- `rover-MinFixType.lua` used a 0-indexed GPS instance; `ASV_FIX_GPS` is 1-indexed (1 = GPS1).
- Remove `rover-MinFixType.lua` and `rover-TerrainDetector.lua` from the autopilot.

## Development

`.luarc.json` points the Lua language server at ArduPilot's scripting type stubs in a sibling checkout (`../ardupilot/libraries/AP_Scripting/docs/docs.lua`). Clone ArduPilot next to this repo for type checking.
