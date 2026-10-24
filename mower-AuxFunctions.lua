--[[----------------------------------------------------------------------------

Controls parking brake and PTO relays on a mower.

CAUTION: This script is capable of engaging and disengaging autonomous control
of a vehicle.  Use this script AT YOUR OWN RISK.

-- Yuri -- Mar 2023

LICENSE - GNU GPLv3 https://www.gnu.org/licenses/gpl-3.0.en.html
------------------------------------------------------------------------------]]

local PTO_RC_OPTION    = 301   -- RC option for relay control (300, 301, etc)
local PTO_RELAY        = 2     -- relay to control
local BRAKE_SERVO_FN   = 95    -- servo function for parking brake (94, 95, etc)
local BRAKE_DELAY_MS   = 2000  -- how long to delay parking brake actuation
local RUN_INTERVAL_MS  = 100

local Brake_Engage    = function() SRV_Channels:set_output_norm(BRAKE_SERVO_FN, 1) return true end
local Brake_Disengage = function() SRV_Channels:set_output_norm(BRAKE_SERVO_FN, -1) return false end
local PTO_Engage      = function() relay:on(PTO_RELAY) return true end
local PTO_Disengage   = function() relay:off(PTO_RELAY) return false end

local ROVER_MODE      =  { HOLD = 4 }
local MAV_SEVERITY         = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }

assert(SRV_Channels:find_channel(BRAKE_SERVO_FN), 'Brake servo unassigned!')
local pto_chan = assert(rc:find_channel_for_option(PTO_RC_OPTION), 'PTO switch unassigned!')
local brake_engaged = false
local last_pto_sw_pos = 0
local last_mode = vehicle:get_mode()
local last_arming_state = arming:is_armed()

function park()
    if arming:is_armed() and vehicle:get_mode() ~= ROVER_MODE.HOLD then
        return update, RUN_INTERVAL_MS
    end
    if brake_engaged then return update, RUN_INTERVAL_MS end

    brake_engaged = Brake_Engage()
    gcs:send_text(MAV_SEVERITY.NOTICE, 'Parking Brake On')
    return update, RUN_INTERVAL_MS
end

function update()
    local mode = vehicle:get_mode()
    local armed = arming:is_armed()
    local pto_sw_pos = pto_chan:get_aux_switch_pos()

    if pto_sw_pos ~= last_pto_sw_pos then
        if pto_sw_pos > 0 and armed then
            PTO_Engage()
            gcs:send_text(MAV_SEVERITY.NOTICE, 'PTO Engaged')
        else
            PTO_Disengage()
            gcs:send_text(MAV_SEVERITY.NOTICE, 'PTO Disengaged')
        end
        last_pto_sw_pos = pto_sw_pos
    end

    if (last_mode ~= mode and mode == ROVER_MODE.HOLD) or
       (last_arming_state and not armed) then
        last_arming_state = armed
        last_mode = mode
        return park, BRAKE_DELAY_MS
    end

    if armed and brake_engaged then
        if (last_mode == ROVER_MODE.HOLD or not last_arming_state) and
           mode ~= ROVER_MODE.HOLD then
            brake_engaged = Brake_Disengage()
            gcs:send_text(MAV_SEVERITY.NOTICE, 'Parking Brake Off')
        end
    end

    last_arming_state = armed
    last_mode = mode
    return update, RUN_INTERVAL_MS
end

gcs:send_text(MAV_SEVERITY.INFO, 'Parking Brake/PTO: Scripted')

return park, BRAKE_DELAY_MS
