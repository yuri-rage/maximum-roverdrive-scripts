--[[----------------------------------------------------------------------------

AutoSupervisor ArduPilot Lua script
(combines the former MinFixType and TerrainDetector scripts)

During auto missions:
  1. GPS fix monitor: pauses the mission when the GPS fix type stays below a
     threshold, then resumes once a good fix has been present long enough.
  2. Terrain detector: uses gyro and accelerometer data to detect rough
     terrain and slow the vehicle, restoring the mission's speed afterwards
     (WP_SPEED, or the latest DO_CHANGE_SPEED in the mission).
  3. Mission speed restore: re-entering AUTO (e.g. resuming after a GPS
     pause) resets the speed to WP_SPEED; a DO_CHANGE_SPEED from an earlier
     leg is restored.

Speed is commanded only to slow down for rough terrain or to resume normal
speed. If rough terrain was detected within ASV_TRN_TIME before a GPS pause,
the vehicle is slowed again after resuming (AUTO re-entry resets the speed).

Parameters:
  ASV_VERBOSE      0: errors only, 1: normal, 2: debug
  ASV_FIX_GPS      GPS instance to monitor (1 = GPS1)
  ASV_FIX_MIN      min GPS fix type (6 = RTK Fixed); 0 disables the monitor
  ASV_FIX_BAD_T    (s) bad fix must persist this long before pausing
  ASV_FIX_GOOD_T   (s) good fix must persist this long before resuming
  ASV_FIX_PAUSE    mode used to pause the mission (0 = MANUAL)
  ASV_FIX_THRMODE  mode used instead if throttle is not neutral (4 = HOLD)
  ASV_TRN_SPD      (m/s) rough terrain speed; -1 disables
                   0 or > mission speed = half mission speed
  ASV_TRN_GZ       (G) slow down above this vertical impulse
  ASV_TRN_GYRO     (deg/s) slow down above this roll/pitch rate
  ASV_TRN_TIME     (s) min time to remain slowed down
  ASV_TRN_GZ_GAIN  multiplier on ASV_TRN_GZ while slowed (<1 is stickier)
  ASV_TRN_GY_GAIN  multiplier on ASV_TRN_GYRO while slowed (<1 is stickier)

Concept for terrain detection first presented during this live stream:
https://www.youtube.com/watch?v=UdXGXjigxAo&t=7155s
Credit to @ktrussell for the idea and discussion!

Requires modules/roverdrive_util.lua.

CAUTION: This script is capable of engaging and disengaging autonomous control
of a vehicle.  Use this script AT YOUR OWN RISK.

-- Yuri -- Aug 2021 (MinFixType), Jun 2022 (TerrainDetector), combined Nov 2026

LICENSE - GNU GPLv3 https://www.gnu.org/licenses/gpl-3.0.en.html
------------------------------------------------------------------------------]]

local util = require('roverdrive_util')

local SCRIPT_NAME         = 'ASV'
local ACTIVE_INTERVAL_MS  = 25  -- needs to be fast for good terrain detection
local STANDBY_INTERVAL_MS = 200
local PARAM_TABLE_KEY     = 62  -- must be unique among scripts on the vehicle

local P = util.param_table(PARAM_TABLE_KEY, 'ASV_', {
    { 'VERBOSE',  2 },
    { 'FIX_GPS',  1 },
    { 'FIX_MIN',  6 },
    { 'FIX_BAD_T', 1.6 },
    { 'FIX_GOOD_T', 0.6 },
    { 'FIX_PAUSE', util.ROVER_MODE.MANUAL },
    { 'FIX_THRMODE', util.ROVER_MODE.HOLD },
    { 'TRN_SPD',  0 },
    { 'TRN_GZ',   1.33 },
    { 'TRN_GYRO', 28 },
    { 'TRN_TIME', 7.5 },
    { 'TRN_GZ_GAIN', 0.9 },
    { 'TRN_GY_GAIN', 0.8 },
})

local log        = util.logger(SCRIPT_NAME, P.VERBOSE)
local ROVER_MODE = util.ROVER_MODE

local G                       = -9.81 -- m/s/s (body frame z is down)
local MAV_CMD_DO_CHANGE_SPEED = 178
local MISSION_SCAN_PER_TICK   = 10

-- mirrors AP_Mission::is_nav_cmd()
local function is_nav_cmd(cmd)
    return cmd <= 95 or      -- MAV_CMD_NAV_LAST
        cmd == 213 or        -- MAV_CMD_NAV_SET_YAW_SPEED
        cmd == 42702 or      -- MAV_CMD_NAV_SCRIPT_TIME
        cmd == 42703         -- MAV_CMD_NAV_ATTITUDE_TIME
end

local FIX_TYPES = {
    [0] = 'No GPS',
    [1] = 'No Fix',
    [2] = '2D Fix',
    [3] = '3D Fix',
    [4] = 'DGPS Fix',
    [5] = 'RTK Float',
    [6] = 'RTK Fixed',
    [7] = 'Static Fixed',
    [8] = 'PPP, 3D',
}

local function fix_name(fix_type)
    return FIX_TYPES[fix_type] or ('fix type ' .. tostring(fix_type))
end

-------------------------------------------------------------------------------
-- RC helpers (read at boot; reboot after changing these params)
-------------------------------------------------------------------------------

local MODE_THRESHOLDS = { 1231, 1361, 1491, 1621, 1750 }
local USER_MODES = {}
for i = 1, 6 do USER_MODES[i] = param:get(string.format('MODE%d', i)) end

-- Parameter(name) raises an error at load if the parameter does not exist
local MODE_CH  = math.floor(Parameter('MODE_CH'):get())
local THR_CH   = math.floor(Parameter('RCMAP_THROTTLE'):get())
local THR_TRIM = Parameter(string.format('RC%d_TRIM', THR_CH)):get()
local THR_DZ   = Parameter(string.format('RC%d_DZ', THR_CH)):get()
local WP_SPEED = Parameter('WP_SPEED')

-- returns the mode selected by the RC mode switch, or nil if the input is invalid
local function get_user_mode()
    local pwm = rc:get_pwm(MODE_CH)
    if not pwm or pwm <= 900 or pwm >= 2200 then return nil end
    for i, threshold in ipairs(MODE_THRESHOLDS) do
        if pwm < threshold then return USER_MODES[i] end
    end
    return USER_MODES[6]
end

local function throttle_neutral()
    local pwm = rc:get_pwm(THR_CH)
    return pwm ~= nil and math.abs(pwm - THR_TRIM) <= THR_DZ
end

-------------------------------------------------------------------------------
-- GPS fix monitor
-------------------------------------------------------------------------------

local fix = {
    paused     = false,
    bad_since  = nil, -- millis() when the fix first went bad
    good_since = nil, -- millis() when the fix first went good
}

local function pause_mode()
    if throttle_neutral() then return math.floor(P.FIX_PAUSE:get()) end
    return math.floor(P.FIX_THRMODE:get())
end

local function end_pause(reason)
    log.warn('Pause canceled - %s', reason)
    fix.paused = false
end

local function fix_update(now, running)
    local min_fix = P.FIX_MIN:get()
    if min_fix <= 0 then
        fix.paused, fix.bad_since, fix.good_since = false, nil, nil
        return
    end

    local fix_type = gps:status(math.floor(P.FIX_GPS:get()) - 1)
    if fix_type >= min_fix then
        fix.bad_since  = nil
        fix.good_since = fix.good_since or now
    else
        if not fix.bad_since and running then
            log.debug('GPS%d - %s', math.floor(P.FIX_GPS:get()), fix_name(fix_type))
        end
        fix.good_since = nil
        fix.bad_since  = fix.bad_since or now
    end

    if not fix.paused then
        if running and fix.bad_since and now - fix.bad_since >= P.FIX_BAD_T:get() * 1000 then
            local mode = pause_mode()
            if mode ~= math.floor(P.FIX_PAUSE:get()) then log.warn('Throttle not neutral!') end
            vehicle:set_mode(mode)
            log.warn('Mission paused - %s', fix_name(fix_type))
            fix.paused = true
        end
        return
    end

    -- paused: the RC mode switch must stay on AUTO for the pause to remain active
    if get_user_mode() ~= ROVER_MODE.AUTO then return end_pause('mode switch') end
    if not arming:is_armed() then return end_pause('disarmed') end

    if fix.good_since and now - fix.good_since >= P.FIX_GOOD_T:get() * 1000 then
        if vehicle:set_mode(ROVER_MODE.AUTO) then
            log.notice('Mission resumed')
            fix.paused = false
        end
        return
    end

    local cur_mode    = vehicle:get_mode()
    local pause_m     = math.floor(P.FIX_PAUSE:get())
    local safeguard_m = math.floor(P.FIX_THRMODE:get())

    if cur_mode == safeguard_m and pause_m ~= safeguard_m then
        -- move from the throttle safeguard mode once the throttle is centered
        if throttle_neutral() then
            log.notice('Throttle neutral')
            vehicle:set_mode(pause_m)
        end
    elseif cur_mode == ROVER_MODE.AUTO then
        -- a non-RC command tried to resume while the fix is still unsuitable
        log.warn('Cannot resume - %s', fix_name(fix_type))
        vehicle:set_mode(pause_mode())
    elseif cur_mode ~= pause_m then
        -- deliberate change to some other mode (e.g. from a GCS)
        end_pause('mode change')
    end
end

-------------------------------------------------------------------------------
-- mission speed tracking (WP_SPEED, or the latest executed DO_CHANGE_SPEED)
-------------------------------------------------------------------------------

local spd = {
    next_idx        = 1,     -- next mission index to inspect (0 is home)
    last_nav        = 0,     -- nav index seen on the previous scan
    speed           = nil,   -- latest DO_CHANGE_SPEED value, if any
    speed_idx       = nil,   -- mission index of that DO_CHANGE_SPEED
    changed         = false, -- a new DO_CHANGE_SPEED was found
    caught_up       = false, -- scan has reached the current mission position
    restore_pending = false, -- AUTO was (re)entered; check for a lost speed
    resume_nav      = 0,     -- nav index when AUTO was (re)entered
}

local function mission_speed_reset()
    spd.next_idx, spd.last_nav, spd.speed, spd.speed_idx = 1, 0, nil, nil
    spd.caught_up = false
end

-- incrementally scans the mission for DO_CHANGE_SPEED commands that have run:
-- those before the current nav command and the do-commands that follow it
local function mission_speed_update()
    local cur_nav = mission:get_current_nav_index()
    if cur_nav < spd.last_nav then
        mission_speed_reset() -- DO_JUMP or restart: rescan from the beginning
    end
    spd.last_nav = cur_nav

    local total = mission:num_commands()
    spd.caught_up = false
    for _ = 1, MISSION_SCAN_PER_TICK do
        if spd.next_idx >= total then spd.caught_up = true; break end
        local item = mission:get_item(spd.next_idx)
        if not item then break end
        local cmd = item:command()
        if spd.next_idx > cur_nav and is_nav_cmd(cmd) then -- next nav cmd not started
            spd.caught_up = true
            break
        end
        if cmd == MAV_CMD_DO_CHANGE_SPEED and item:param2() > 0 then
            if item:param2() ~= spd.speed then spd.changed = true end
            spd.speed     = item:param2()
            spd.speed_idx = spd.next_idx
        end
        spd.next_idx = spd.next_idx + 1
    end
end

local function mission_speed()
    return spd.speed or WP_SPEED:get()
end

-------------------------------------------------------------------------------
-- terrain detector
-------------------------------------------------------------------------------

local trn = {
    last_g_z     = nil,   -- previous Gz sample, for impulse calculation
    last_bump_ms = nil,   -- millis() of the most recent rough terrain detection
    slowed       = false, -- true while the rough terrain speed is in effect
    max_impulse  = 0,     -- debug stats
    max_rate     = 0,
    warned_speed = false,
}

-- mission stopped or paused: AUTO re-entry restores the default speed,
-- so the vehicle is no longer slowed
local function terrain_reset()
    trn.last_g_z     = nil
    trn.slowed       = false
    trn.warned_speed = false
end

local function rough_speed(normal)
    local rough = P.TRN_SPD:get()
    if rough > normal and not trn.warned_speed then
        log.warn('ASV_TRN_SPD > mission spd - using half')
        trn.warned_speed = true
    end
    -- unset or invalid: use half speed
    if rough == 0 or rough > normal then return normal / 2 end
    return rough
end

-- speed is commanded only to slow down or to resume normal speed
local function terrain_in_rough(now)
    return P.TRN_SPD:get() >= 0 and trn.last_bump_ms ~= nil and
        now - trn.last_bump_ms < P.TRN_TIME:get() * 1000
end

-- min groundspeed for commanding a speed change (avoids pivot turns)
-- half of the slower speed seems adequate
local function speed_change_min()
    local normal = mission_speed()
    local rough  = P.TRN_SPD:get()
    if rough <= 0 or rough > normal then rough = normal / 2 end
    return rough / 2
end

local function terrain_update(now, running)
    if not running or P.TRN_SPD:get() < 0 then
        terrain_reset()
        return
    end

    -- a DO_CHANGE_SPEED executed while slowed overrides the rough speed
    local reslow = spd.changed and trn.slowed
    spd.changed = false

    local normal  = mission_speed()
    local rough   = rough_speed(normal)
    local in_rough = terrain_in_rough(now)

    -- sample sensors
    local g_z       = ahrs:get_accel():z() / G -- convert to G from m/s/s
    local gyro      = ahrs:get_gyro()
    local gyro_rate = math.deg(math.max(math.abs(gyro:x()), math.abs(gyro:y()))) -- max of roll/pitch rates
    local impulse   = trn.last_g_z and math.abs(g_z - trn.last_g_z) or 0 -- sample delta is the impulse
    trn.last_g_z    = g_z

    trn.max_impulse = math.max(trn.max_impulse, impulse)
    trn.max_rate    = math.max(trn.max_rate, gyro_rate)

    local gz_threshold   = P.TRN_GZ:get() * (in_rough and P.TRN_GZ_GAIN:get() or 1)
    local gyro_threshold = P.TRN_GYRO:get() * (in_rough and P.TRN_GY_GAIN:get() or 1)
    if impulse > gz_threshold then
        log.debug('Impulse - %.2f Gz', impulse)
        trn.last_bump_ms = now
        in_rough = true
    elseif gyro_rate > gyro_threshold then
        log.debug('Transient - %.2f deg/s', gyro_rate)
        trn.last_bump_ms = now
        in_rough = true
    end

    if in_rough == trn.slowed and not reslow then return end

    -- avoid commanding a speed change when slow (like during a pivot turn)
    if ahrs:groundspeed_vector():length() < speed_change_min() then
        if reslow then spd.changed = true end -- retry on a later tick
        return
    end

    local target = in_rough and rough or normal
    if not vehicle:set_desired_speed(target) then
        log.debug('Speed change to %.2f m/s rejected', target)
        if reslow then spd.changed = true end
        return
    end
    if in_rough then
        log.warn('Slowing for rough terrain')
    else
        log.notice('Resuming normal speed')
        log.debug('Max - %.2f Gz ... %.2f deg/s', trn.max_impulse, trn.max_rate)
    end
    trn.max_impulse, trn.max_rate = 0, 0
    trn.slowed = in_rough
end

-------------------------------------------------------------------------------
-- mission speed restore
-- AUTO re-entry resets the speed to WP_SPEED and only re-runs do-commands
-- that follow the current nav command, so a DO_CHANGE_SPEED from an earlier
-- leg is lost. Restore it once the scan has caught up.
-------------------------------------------------------------------------------

local function speed_restore_update(now, running)
    if not running or not spd.restore_pending or not spd.caught_up then return end

    local lost = spd.speed ~= nil and spd.speed_idx < spd.resume_nav
    if not lost or trn.slowed or terrain_in_rough(now) then
        -- nothing lost, or the terrain detector owns the speed and will
        -- resume to the mission speed when it is done
        spd.restore_pending = false
        return
    end

    if ahrs:groundspeed_vector():length() < speed_change_min() then return end

    if vehicle:set_desired_speed(spd.speed) then
        log.notice('Mission speed restored - %.2f m/s', spd.speed)
        spd.restore_pending = false
    end
end

-------------------------------------------------------------------------------
-- main loop
-------------------------------------------------------------------------------

local was_running = false

local function update()
    local now     = millis()
    local running = mission:state() == mission.MISSION_RUNNING

    if not running and not fix.paused then
        mission_speed_reset() -- re-derived on the next mission start/resume
    end
    if running and not was_running then
        spd.restore_pending = true
        spd.resume_nav      = mission:get_current_nav_index()
    end
    was_running = running

    fix_update(now, running)
    if running then mission_speed_update() end
    terrain_update(now, running)
    speed_restore_update(now, running)

    -- re-check: fix_update() may have just resumed the mission
    if fix.paused or mission:state() == mission.MISSION_RUNNING then
        return update, ACTIVE_INTERVAL_MS
    end
    return update, STANDBY_INTERVAL_MS
end

log.info('Script active')

return util.run(SCRIPT_NAME, update)()
