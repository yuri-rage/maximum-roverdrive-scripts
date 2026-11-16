--[[----------------------------------------------------------------------------

roverdrive_util ArduPilot Lua module

Shared helpers for the Maximum Roverdrive scripts. Install to the autopilot's
APM/scripts/modules/ directory and load with:
    local util = require('roverdrive_util')

-- Yuri -- Nov 2026

LICENSE - GNU GPLv3 https://www.gnu.org/licenses/gpl-3.0.en.html
------------------------------------------------------------------------------]]

local M = {}

M.MAV_SEVERITY = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }

M.ROVER_MODE = {
    MANUAL    = 0,
    ACRO      = 1,
    STEERING  = 3,
    HOLD      = 4,
    LOITER    = 5,
    FOLLOW    = 6,
    SIMPLE    = 7,
    DOCK      = 8,
    CIRCLE    = 9,
    AUTO      = 10,
    RTL       = 11,
    SMART_RTL = 12,
    GUIDED    = 15,
}

-- verbosity levels (compared against a script's *_VERBOSE parameter)
M.MSG_ALWAYS = 0 -- errors: sent even when verbosity is 0
M.MSG_NORMAL = 1
M.MSG_DEBUG  = 2

-- returns a logger that prefixes messages with script_name
-- verbosity: a bound Parameter, a number, or nil (normal)
-- usage: log.info('WP %d saved', idx)
function M.logger(script_name, verbosity)
    local function level()
        if type(verbosity) == 'number' then return verbosity end
        if verbosity then return verbosity:get() end
        return M.MSG_NORMAL
    end

    local function send(min_level, severity, fmt, ...)
        if level() >= min_level then
            gcs:send_text(severity, script_name .. ': ' .. string.format(fmt, ...))
        end
    end

    return {
        info   = function(fmt, ...) send(M.MSG_NORMAL, M.MAV_SEVERITY.INFO, fmt, ...) end,
        notice = function(fmt, ...) send(M.MSG_NORMAL, M.MAV_SEVERITY.NOTICE, fmt, ...) end,
        warn   = function(fmt, ...) send(M.MSG_NORMAL, M.MAV_SEVERITY.WARNING, fmt, ...) end,
        error  = function(fmt, ...) send(M.MSG_ALWAYS, M.MAV_SEVERITY.ERROR, fmt, ...) end,
        debug  = function(fmt, ...) send(M.MSG_DEBUG, M.MAV_SEVERITY.INFO, fmt, ...) end,
    }
end

-- adds a scripting parameter table and binds each parameter
-- defs: ordered list of { name, default } (name excludes the prefix)
-- returns a table of bound Parameter objects keyed by name, e.g. P.MIN_DIST:get()
-- key must be unique among all scripts running on the vehicle
---@param key integer
---@param prefix string
---@param defs table
---@return table<string, Parameter_ud_const>
function M.param_table(key, prefix, defs)
    assert(param:add_table(key, prefix, #defs), string.format('could not add %s param table', prefix))
    local params = {}
    for idx, def in ipairs(defs) do
        local name, default = def[1], def[2]
        assert(param:add_param(key, idx, name, default), string.format('could not add %s%s', prefix, name))
        params[name] = Parameter(prefix .. name) -- raises an error if not found
    end
    return params
end

-- runs a state machine whose state functions return (next_state, delay_ms)
-- runtime errors are reported and the machine restarts at opts.reset (default: initial)
-- opts.on_error: optional fail-safe callback run after an error
-- returns the scheduler function; scripts end with: return util.run(...)()
function M.run(script_name, initial, opts)
    opts = opts or {}
    local state = initial
    local function loop()
        local ok, next_state, delay_ms = pcall(state)
        if not ok then
            gcs:send_text(M.MAV_SEVERITY.ERROR, script_name .. ': Internal error: ' .. tostring(next_state))
            if opts.on_error then pcall(opts.on_error) end
            state = opts.reset or initial
            return loop, opts.error_delay_ms or 1000
        end
        state = next_state
        return loop, delay_ms
    end
    return loop
end

return M
