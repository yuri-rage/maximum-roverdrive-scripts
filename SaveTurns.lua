--[[----------------------------------------------------------------------------

SaveTurns ArduPilot Lua script

Saves the locations of vehicle turns as waypoints in the current nav mission.

RC switch (STRN_RC_OPT):
    LOW  - standby
    MID  - record turns (the starting position is saved first)
    HIGH - clear the mission

Parameters (reboot after changing STRN_RC_OPT):
    STRN_RC_OPT    RC aux option for the 3-position switch (300-307)
    STRN_MIN_DIST  (m)   min distance between waypoints
    STRN_HDG_DELTA (deg) save a waypoint after a heading change of this size
    STRN_VERBOSE   0: errors only, 1: normal, 2: debug

Requires modules/roverdrive_util.lua.

CAUTION: This script is capable of engaging and disengaging autonomous control
of a vehicle.  Use this script AT YOUR OWN RISK.

-- Yuri -- Nov 2021, revised Apr 2022, Nov 2026

LICENSE - GNU GPLv3 https://www.gnu.org/licenses/gpl-3.0.en.html
------------------------------------------------------------------------------]]

local util = require('roverdrive_util')

local SCRIPT_NAME     = 'SaveTurns'
local RUN_INTERVAL_MS = 200
local PARAM_TABLE_KEY = 60 -- must be unique among scripts on the vehicle

local P = util.param_table(PARAM_TABLE_KEY, 'STRN_', {
    { 'RC_OPT',    303 },
    { 'MIN_DIST',  2.0 },
    { 'HDG_DELTA', 8.0 },
    { 'VERBOSE',   1 },
})

local log                  = util.logger(SCRIPT_NAME, P.VERBOSE)
local ROVER_MODE           = util.ROVER_MODE
local SWITCH               = { STANDBY = 0, SAVE = 1, CLEAR = 2 }
local MAV_CMD_NAV_WAYPOINT = 16

local rc_chan = rc:find_channel_for_option(math.floor(P.RC_OPT:get()))
if not rc_chan then
    log.error('RC option %d not found', math.floor(P.RC_OPT:get()))
    return
end

local last_wp  = nil -- Location of the last saved waypoint (nil until one is saved)
local last_yaw = nil -- (deg) heading when departing last_wp

-- forward declarations for state functions
local initialize, standby, recording, await_switch_change
local hold_pos = nil -- switch position to wait out in await_switch_change

local function yaw_diff(yaw1, yaw2)
    --https://stackoverflow.com/questions/5024375/getting-the-difference-between-two-headings
    return math.abs((yaw2 - yaw1 + 540.0) % 360.0 - 180.0)
end

local function waypoint_item(lat, lng, alt_m)
    local item = mavlink_mission_item_int_t()
    item:command(MAV_CMD_NAV_WAYPOINT)
    item:x(lat)
    item:y(lng)
    item:z(alt_m)
    return item
end

-- clears the mission, leaving only the home item
local function new_mission()
    if not ahrs:home_is_set() then
        log.warn('Home not set - cannot create mission')
        return false
    end
    local home = ahrs:get_home()
    mission:clear()
    if not mission:set_item(0, waypoint_item(home:lat(), home:lng(), home:alt() * 0.01)) then
        log.warn('Failed to create new mission')
        return false
    end
    return true
end

local function save_wp(position)
    local idx = mission:num_commands()
    if not mission:set_item(idx, waypoint_item(position:lat(), position:lng(), 0)) then
        log.warn('WP %d - failed to save', idx)
        return false
    end
    log.info('WP %d saved', idx)
    return true
end

local function start_recording()
    -- index 0 is reserved for home; never save a turn there
    if mission:num_commands() == 0 and not new_mission() then
        return false
    end
    last_wp  = nil
    last_yaw = nil
    log.info('Recording turns')
    return true
end

recording = function()
    if vehicle:get_mode() == ROVER_MODE.AUTO then
        log.warn('AUTO mode - recording canceled')
        return standby, RUN_INTERVAL_MS
    end
    if rc_chan:get_aux_switch_pos() ~= SWITCH.SAVE then
        log.info('Recording stopped')
        return standby, RUN_INTERVAL_MS
    end
    if not ahrs:healthy() then return recording, RUN_INTERVAL_MS end

    local cur_pos = ahrs:get_location()
    if not cur_pos then return recording, RUN_INTERVAL_MS end
    local cur_yaw = math.deg(ahrs:get_yaw_rad())

    -- first point of a recording session
    if not last_wp then
        if save_wp(cur_pos) then
            last_wp  = cur_pos
            last_yaw = cur_yaw
        end
        return recording, RUN_INTERVAL_MS
    end

    -- track departure heading until clear of the last waypoint
    if cur_pos:get_distance(last_wp) < P.MIN_DIST:get() then
        last_yaw = cur_yaw
        return recording, RUN_INTERVAL_MS
    end

    if yaw_diff(cur_yaw, last_yaw) > P.HDG_DELTA:get() and save_wp(cur_pos) then
        last_wp  = cur_pos
        last_yaw = cur_yaw
    end

    return recording, RUN_INTERVAL_MS
end

-- after a clear (or failed start), wait for the switch to move before acting again
await_switch_change = function()
    if rc_chan:get_aux_switch_pos() == hold_pos then
        return await_switch_change, RUN_INTERVAL_MS
    end
    return standby()
end

standby = function()
    if vehicle:get_mode() == ROVER_MODE.AUTO then return standby, RUN_INTERVAL_MS end

    local sw_pos = rc_chan:get_aux_switch_pos()
    if sw_pos == SWITCH.SAVE then
        if start_recording() then return recording, RUN_INTERVAL_MS end
        hold_pos = SWITCH.SAVE
        return await_switch_change, RUN_INTERVAL_MS
    end
    if sw_pos == SWITCH.CLEAR then
        if new_mission() then log.warn('Mission cleared') end
        hold_pos = SWITCH.CLEAR
        return await_switch_change, RUN_INTERVAL_MS
    end
    return standby, RUN_INTERVAL_MS
end

initialize = function()
    if not ahrs:healthy() then return initialize, RUN_INTERVAL_MS end
    log.info('Ready')
    return standby, RUN_INTERVAL_MS
end

log.info('Awaiting AHRS initialization...')

return util.run(SCRIPT_NAME, initialize, { reset = standby })()
