--[[----------------------------------------------------------------------------

Controls parking brake and PTO relays on a mower.

PTO:
    Engages when the PTO switch is raised while armed.
    Disengages when the switch is lowered, on disarm, or on crash detection.
    After a forced disengage, cycle the switch to re-engage.

Parking brake:
    Engages MOW_BRK_DELAY seconds after disarming or entering HOLD.
    Never engaged by crash detection itself: the wheels may still be driven to
    recover (e.g. stuck in mud). Rover's crash failsafe enters HOLD, so the brake
    engages after the delay, then releases as soon as a driving mode is selected.
    Releases when armed in any mode other than HOLD.

Parameters (reboot after changing):
    MOW_PTO_RC_OPT  RC aux option for the PTO switch (300-307)
    MOW_PTO_RELAY   PTO relay number (1 = RELAY1); RELAYn_FUNCTION should be 1 (Relay)
    MOW_BRK_SRV_FN  servo function for the parking brake (94-109, Script1-16)
    MOW_BRK_DELAY   (s) delay before engaging the parking brake

Requires modules/roverdrive_util.lua.

CAUTION: This script is capable of engaging and disengaging autonomous control
of a vehicle.  Use this script AT YOUR OWN RISK.

-- Yuri -- Mar 2023, revised Nov 2026

LICENSE - GNU GPLv3 https://www.gnu.org/licenses/gpl-3.0.en.html
------------------------------------------------------------------------------]]

local util = require('roverdrive_util')

local SCRIPT_NAME     = 'MowerAux'
local RUN_INTERVAL_MS = 100
local PARAM_TABLE_KEY = 61 -- must be unique among scripts on the vehicle

local P = util.param_table(PARAM_TABLE_KEY, 'MOW_', {
    { 'PTO_RC_OPT', 301 },
    { 'PTO_RELAY',  3 },
    { 'BRK_SRV_FN', 95 },
    { 'BRK_DELAY',  2.0 },
})

local log        = util.logger(SCRIPT_NAME)
local ROVER_MODE = util.ROVER_MODE

local PTO_RELAY      = math.floor(P.PTO_RELAY:get())
local BRAKE_SERVO_FN = math.floor(P.BRK_SRV_FN:get())

-- relay:on/off() only drive relays with RELAYn_FUNCTION = 1 (Relay)
-- warn rather than fail: brake/HOLD handling must keep working
-- (firmware without RELAYn_FUNCTION params returns nil here)
local relay_fn     = param:get(string.format('RELAY%d_FUNCTION', PTO_RELAY))
local pto_relay_ok = relay_fn == nil or relay_fn == 1
if not pto_relay_ok then
    log.error('RELAY%d_FUNCTION is not 1 (Relay) - PTO will not operate', PTO_RELAY)
end
assert(SRV_Channels:find_channel(BRAKE_SERVO_FN),
    string.format('%s: no servo assigned to function %d', SCRIPT_NAME, BRAKE_SERVO_FN))
local pto_chan = assert(rc:find_channel_for_option(math.floor(P.PTO_RC_OPT:get())),
    string.format('%s: PTO switch unassigned', SCRIPT_NAME))

local pto_engaged     = false
local brake_engaged   = nil -- nil = unknown (forces a command on first update)
local park_request_ms = nil -- time a park condition was first seen
local last_pto_sw_pos = 0

local function set_pto(engage, reason)
    if engage and not pto_relay_ok then
        log.error('RELAY%d_FUNCTION is not 1 (Relay) - PTO will not operate', PTO_RELAY)
    end
    if engage then
        relay:on(PTO_RELAY - 1) -- Lua relay instances are 0-indexed
    else
        relay:off(PTO_RELAY - 1)
    end
    if engage ~= pto_engaged then
        log.notice('PTO %s%s', engage and 'engaged' or 'disengaged', reason and (' - ' .. reason) or '')
    end
    pto_engaged = engage
end

local function set_brake(engage)
    SRV_Channels:set_output_norm(BRAKE_SERVO_FN, engage and 1 or -1)
    log.notice('Parking brake %s', engage and 'on' or 'off')
    brake_engaged = engage
end

local function update()
    local now     = millis()
    local armed   = arming:is_armed()
    local crashed = vehicle:is_crashed()
    local mode    = vehicle:get_mode()

    -- PTO: forced off on disarm/crash, otherwise follows switch changes
    local pto_sw_pos = pto_chan:get_aux_switch_pos()
    if pto_engaged and not armed then
        set_pto(false, 'disarmed')
    elseif pto_engaged and crashed then
        set_pto(false, 'crash')
    elseif pto_sw_pos ~= last_pto_sw_pos then
        set_pto(pto_sw_pos > 0 and armed and not crashed)
    end
    last_pto_sw_pos = pto_sw_pos

    -- parking brake
    -- intentionally independent of crash state (see header)
    local park = (not armed) or mode == ROVER_MODE.HOLD
    if park then
        if brake_engaged ~= true then
            park_request_ms = park_request_ms or now
            if now - park_request_ms >= P.BRK_DELAY:get() * 1000 then
                set_brake(true)
            end
        end
    else
        park_request_ms = nil
        if brake_engaged ~= false then set_brake(false) end
    end

    return update, RUN_INTERVAL_MS
end

-- fail safe on internal error: kill the PTO
local function on_error()
    relay:off(PTO_RELAY - 1)
    pto_engaged = false
end

relay:off(PTO_RELAY - 1) -- known state at startup
log.info('Parking brake/PTO active')

return util.run(SCRIPT_NAME, update, { on_error = on_error })()
