--[[----------------------------------------------------------------------------

TerrainDetector ArduPilot Lua script

Uses gyro and accelerometer data to detect rough terrain and slow the
vehicle's speed accordingly during auto missions.

CAUTION: This script is capable of engaging and disengaging autonomous control
of a vehicle.  Use this script AT YOUR OWN RISK.

-- Yuri -- Jun 2022

LICENSE - GNU GPLv3 https://www.gnu.org/licenses/gpl-3.0.en.html

Concept first presented during this live stream:
https://www.youtube.com/watch?v=UdXGXjigxAo&t=7155s
Credit to @ktrussell for the idea and discussion!
------------------------------------------------------------------------------]]

local SCRIPT_NAME     = 'TerrainDetector'
local RUN_INTERVAL_MS = 25 -- needs to be pretty fast for good detection
local SBY_INTERVAL_MS = 500 -- slower interval when detection is disabled

-- tuning values
local SCR_USER_SPEED_PARAM     = 'SCR_USER1' -- (m/s) set rough terrain speed here
-- set to -1 to disable detection
local IMPULSE_GZ_THRESHOLD     = 1.33 -- (G) slow down > this Gz impulse
local GYRO_RATE_THRESHOLD      = 28 -- (deg/s) slow down > this rate transient
local ROUGH_TERRAIN_TIMEOUT_MS = 7500 -- min time to remain slowed down for rough terrain

-- gain values are used for smooth terrain detection
-- set less than 1.0 to be more restrictive about resuming normal speed
local IMPULSE_GAIN = 0.9
local GYRO_GAIN    = 0.8

-- GCS messages
local VERBOSE_MODE = 2 -- 0 to suppress all GCS messages,
-- 1 for normal status messages
-- 2 for additional debug messages
local MSG_NORMAL   = 1
local MSG_DEBUG    = 2

-- MAVLink values
local MAV_SEVERITY_WARNING = 4
local MAV_SEVERITY_INFO    = 6

-- mathematical/physical constants
local G          = -9.81 -- m/s/s
local RAD_TO_DEG = 180.0 / math.pi

-- wrapper for gcs:send_text()
local function gcs_msg(msg_type, severity, txt)
    if type(msg_type) == 'string' then
        -- allow just a string to be passed for simple/routine messages
        txt      = msg_type
        msg_type = MSG_NORMAL
        severity = MAV_SEVERITY_INFO
    end
    if type(severity) == 'string' then
        -- allow just severity and string to be passed for normal messages
        txt = severity
        severity = msg_type
        msg_type = MSG_NORMAL
    end
    if msg_type <= VERBOSE_MODE then
        gcs:send_text(severity, string.format('%s: %s', SCRIPT_NAME, txt))
    end
end

local last_g_z   = 0 -- to calculate impulse Gz
local timeout_ms = 0

-- debug values
local max_g_z       = 0
local max_gyro_rate = 0

local wp_speed_normal = param:get('WP_SPEED')
local wp_speed_rough  = param:get(SCR_USER_SPEED_PARAM)

function update()
    wp_speed_rough = param:get(SCR_USER_SPEED_PARAM)
    if wp_speed_rough < 0 then return update, SBY_INTERVAL_MS end

    if mission:state() == mission.MISSION_RUNNING then
        wp_speed_normal = param:get('WP_SPEED')
        -- if slowdown param not set or invalid, use half of WP_SPEED
        if wp_speed_rough == 0 or wp_speed_rough > wp_speed_normal then
            wp_speed_rough = wp_speed_normal / 2
            gcs_msg(SCR_USER_SPEED_PARAM .. ' speed invalid')
            gcs_msg('Using WP_SPEED / 2')
        end

        last_g_z = 0
        return do_normal_speed, RUN_INTERVAL_MS
    end
    return update, SBY_INTERVAL_MS
end

function initiate_rough_speed()
    -- avoid commanding a speed change when slow (like during a pivot turn)
    -- half of the slower wp speed seems adequate
    if ahrs:groundspeed_vector():length() < wp_speed_rough / 2 then
        return initiate_rough_speed, RUN_INTERVAL_MS
    end
    gcs_msg(MSG_NORMAL, MAV_SEVERITY_WARNING, 'Slowing for rough terrain')
    vehicle:set_desired_speed(wp_speed_rough)
    last_g_z      = 0
    timeout_ms    = millis() + ROUGH_TERRAIN_TIMEOUT_MS
    max_g_z       = 0
    max_gyro_rate = 0
    return do_rough_speed, RUN_INTERVAL_MS
end

function initiate_normal_speed()
    -- avoid commanding a speed change when slow (like during a pivot turn)
    -- half of the slower wp speed seems adequate
    if ahrs:groundspeed_vector():length() < wp_speed_rough / 2 then
        return initiate_rough_speed, RUN_INTERVAL_MS
    end
    gcs_msg(MSG_NORMAL, MAV_SEVERITY_WARNING, 'Resuming normal speed')
    vehicle:set_desired_speed(wp_speed_normal)
    last_g_z = 0
    return do_normal_speed, RUN_INTERVAL_MS
end

function do_normal_speed()
    if mission:state() ~= mission.MISSION_RUNNING then
        return update, SBY_INTERVAL_MS
    end

    local g_z         = ahrs:get_accel():z() / G -- convert to G from m/s/s
    local gyro        = ahrs:get_gyro()
    -- sample max of roll/pitch rates
    local gyro_rate   = math.max(math.abs(gyro:x()), math.abs(gyro:y())) * RAD_TO_DEG
    local impulse_g_z = math.abs(g_z - last_g_z) -- sample delta is the measured impulse
    last_g_z          = g_z

    if impulse_g_z > IMPULSE_GZ_THRESHOLD then -- slow down
        gcs_msg(MSG_DEBUG, MAV_SEVERITY_INFO, string.format('Impulse - %.2f Gz', impulse_g_z))
        return initiate_rough_speed, RUN_INTERVAL_MS
    end

    if gyro_rate > GYRO_RATE_THRESHOLD then -- slow down
        gcs_msg(MSG_DEBUG, MAV_SEVERITY_INFO, string.format('Transient - %.2f deg/s', gyro_rate))
        return initiate_rough_speed, RUN_INTERVAL_MS
    end

    return do_normal_speed, RUN_INTERVAL_MS
end

function do_rough_speed()
    if mission:state() ~= mission.MISSION_RUNNING then
        return update, SBY_INTERVAL_MS
    end

    local g_z         = ahrs:get_accel():z() / G -- convert to G from m/s/s
    local gyro        = ahrs:get_gyro()
    -- sample max of roll/pitch rates
    local gyro_rate   = math.max(math.abs(gyro:x()), math.abs(gyro:y())) * RAD_TO_DEG
    local impulse_g_z = math.abs(g_z - last_g_z) -- sample delta is the measured impulse
    last_g_z          = g_z

    max_g_z = math.max(g_z, max_g_z)
    max_gyro_rate = math.max(gyro_rate, max_gyro_rate)

    local now = millis()

    if impulse_g_z > IMPULSE_GZ_THRESHOLD * IMPULSE_GAIN then -- stay slow
        gcs_msg(MSG_DEBUG, MAV_SEVERITY_INFO, string.format('Timer reset - %.2fGz', impulse_g_z))
        max_g_z = 0
        timeout_ms = now + ROUGH_TERRAIN_TIMEOUT_MS
        return do_rough_speed, RUN_INTERVAL_MS
    end

    if gyro_rate > GYRO_RATE_THRESHOLD * GYRO_GAIN then -- stay slow
        gcs_msg(MSG_DEBUG, MAV_SEVERITY_INFO, string.format('Timer reset - %.2f deg/s', gyro_rate))
        max_gyro_rate = 0
        timeout_ms = now + ROUGH_TERRAIN_TIMEOUT_MS
        return do_rough_speed, RUN_INTERVAL_MS
    end

    if now > timeout_ms then -- no longer in rough terrain
        gcs_msg(MSG_DEBUG, MAV_SEVERITY_INFO, string.format('Max - %.2fGz ... %.2f deg/s', max_g_z, max_gyro_rate))
        return initiate_normal_speed, RUN_INTERVAL_MS
    end

    return do_rough_speed, RUN_INTERVAL_MS
end

gcs_msg('Script active')

return update, SBY_INTERVAL_MS
