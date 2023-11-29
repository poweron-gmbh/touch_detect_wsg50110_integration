--- td grip control
-- @author Sascha Teutoburg-Weiss
-- @usage implements controlled gripping and conditional releasing processes for applications on WSG-50 in combination with td_control_fncs
-- @name td_grip_control
-- version v2
-- 2023-11-14

--[[ OUTSOURCED TD CONTROL AND HELPER FUNCTION MODULE ]] --
local tdcf = require 'td_control_fncs_v2'

--[[ INIT VARIABLES ]]--
-- Script
DEBUG_OUTPUT = false    -- Activate/Deactivate console debug prints
INFO_OUTPUT  = false    -- Activate/Deactivate console info  prints
ALWAYS_RUNNING = true   -- Triggers always running despite as long as client is connected
CMD_UPDATE_MS  = 100    -- Update rate to check for incoming commands
--TD
FINGER_LEFT = 1         -- Index for left finger configuration
FINGER_RIGHT = 0        -- INdex for right finger configuration
TD_INIT_TIME = 6000     -- Time wait for TD initialization - 1/2 after off, 1/2 after init per finger
BAUD_TD = 115200        -- TD baudrate
READ_DATA_COMMAND = {0x7E, 0xFF, 0x12, 0x01, 0x97, 0xB7, 0x7E}      -- Command Structure for TD read sensor values
MASTER_ACK_COMMAND = {0x7E, 0xFF, 0xA1, 0x04, 0x44, 0x7E}           -- Command structure for TD acknowlegment
TIMEOUT_TD_MS = 100     -- Timeout TD read sensor in ms
TIMEOUT_TD_DIVIDER = 10 -- Timeout divider aka sequence-time multiplier
ACK_PACKAGE_SIZE = 84   -- TD package size estimation
--WSG
TIMEOUT_TD_TRY = 10        -- Max tries to read verified sensor values
GRIP_CONTROL_CMD = 0xA0 -- 160 | WSG user specified command for controled gripping
GRIP_RELEASE_CMD = 0xA1 -- 161 | WSG user specified command for releasing (conditional)
PUB_DATA_CMD     = 0xA2 -- 162 | WSG user specified command publishing TD data
-- GRIP_CONTROL
DEFAULT_START_FORCE = 5.0;              -- Default force to start controlled gripping (min 5)
DEFAULT_SPEED = 20;                     -- Default speed to move/release
DEFAULT_START_POSITION = 100.0;         -- Default start position open
DEFAULT_TARGET_POSITION = 10;           -- Default min position target is awaited (max closed)
DEFAULT_MAX_FORCE = 80.0;               -- Default max force in controlled gripping (max 80)
DEFAULT_FORCE_INCREASE_STEP = 2.5;      -- Default steps of increasing force during controlled gripping
DEFAULT_CLAMPING_TRAVEL_INIT = 2;       -- Default first max travel while trying to clamp object during controlled gripping
DEFAULT_CLAMPING_TRAVEL = 2;            -- Default max travel while trying to clamp object in controlled gripping per step

DEFAULT_COND_REL_ACTIVE = false;         -- Default activation state of conditional checks before releasing

DEFAULT_TD_LEVEL = 100;                  -- Default TD threshold level to seen as active node
DEFAULT_TD_IDENTIFICATION_LIMIT = 1     -- Default limit of tries to get "identified/gripped or not" result per step

-- variables
local current_position = 0;
local current_force = 0;
local current_force_limit = 0;
local current_state = "none";
-- grip control
local grip_ok = false;          -- gripping state is ok or not
local identified_r = false;     --
local identified_l = false;     --
local identified = false;       -- Object general present state
local object_clamped = false;   -- object clamped state (gripper still closed/gripping)
local retry = false;

--[[ RUN ]]--
--[[ INITIALIZATION ]]--
-- Configure fingers (both 0 -> 1).
for index = 0, (finger.count() - 1) do
    if finger.type(index) == "generic" then
        -- reset fingers
        if DEBUG_OUTPUT == true then
            printf("...Finger %d : Deactivating \n", index)
        end
        finger.power(index, false)
        sleep(TD_INIT_TIME/2)
        -- Power-up finger
        if DEBUG_OUTPUT == true then
            printf("...Finger %d : Activating \n", index)
        end
        finger.power( index, true )
        -- Configure interface and wait initalization
        if DEBUG_OUTPUT == true then
            printf("...Finger %d : Setting interface \n", index)
        end
        finger.interface( index, "uart", BAUD_TD)
        sleep(TD_INIT_TIME/2)
    end
end

-- start process loop on activated link or plain always
if ((cmd.online() == true) or (ALWAYS_RUNNING == true)) then
    -- Configure gripper
    if DEBUG_OUTPUT == true then
        print("...Initalizing commands, homing and defaults")
    end
    -- register user defined command ids
    cmd.register(GRIP_CONTROL_CMD)
    cmd.register(GRIP_RELEASE_CMD)
    cmd.register(PUB_DATA_CMD)
    -- home gripper and initialize starting points
    mc.homing()
    mc.force(DEFAULT_START_FORCE)
    mc.speed(DEFAULT_SPEED)
    mc.move(DEFAULT_START_POSITION, DEFAULT_SPEED, 0x001)
    if (DEBUG_OUTPUT == true) or (INFO_OUTPUT == true) then
        print("\n Script running.....OK \n");
    end
    -- keeping in process loop
    while (cmd.online() or (ALWAYS_RUNNING == true)) do
        -- wait CMD update cycle
        sleep(CMD_UPDATE_MS);
        if cmd.available() > 0 then
            local id, payload = cmd.read();
            if DEBUG_OUTPUT == true then
                printf("Command incoming with id: %d \n", id)
            end
            -- controlled gripping
            if id == GRIP_CONTROL_CMD then
                if DEBUG_OUTPUT == true then
                    print("...GRIPPING \n")
                end
                grip_ok =false;
                object_clamped = false;
                retry = false;
            
                -- Set defaults + pre-position with wait on contact
                mc.force(DEFAULT_START_FORCE)
                mc.speed(DEFAULT_SPEED)
                --mc.move(DEFAULT_TARGET_POSITION, DEFAULT_SPEED, 0x001)
                mc.move(DEFAULT_TARGET_POSITION, DEFAULT_SPEED,0)
                while mc.busy() do
                    sleep(20)
                end
                -- evaluate target limit <-> actual position
                current_position = mc.position()
                if (DEFAULT_TARGET_POSITION+1.0 > current_position) then
                    -- no part present, identification failed
                    mc.move(DEFAULT_START_POSITION, DEFAULT_SPEED)
                    identified = false;
                    object_clamped = false;
                else
                    -- part present at least
                    identified = true;
                    -- Clamping routine, init loop force
                    local next_force = DEFAULT_START_FORCE
                    while ((grip_ok ~= true) and (next_force <= DEFAULT_MAX_FORCE)) do
                        rertry = false
                        -- update force
                        mc.force(next_force)
                        current_force = next_force
                        if (DEBUG_OUTPUT == true) or (INFO_OUTPUT == true) then
                            printf("Clamping - Force : %.2f \n", current_force)
                            --printf("Clamping - State : %d \n", grasping.state())
                        end
                        -- try clamping
                        grasping.clamp(DEFAULT_CLAMPING_TRAVEL, next_force)
                        sleep(10)
                        object_clamped = true;
                        -- Additional redundancy of data object identification
                        local data_try_count = 0;
                        while ((grip_ok ~= true) and (data_try_count <= DEFAULT_TD_IDENTIFICATION_LIMIT)) do
                            -- Read sensors
                            retry = false
                            local is_top, is_bottom, i_right, i_left, i_any
                            tdcf.get_data_buffered()
                            local data_l = tdcf.data_arr_l()
                            local data_r = tdcf.data_arr_r()
                            -- Check sensor and grip conditions
                            if ((data_l[1] ~= -1) and (data_r[1] ~= -1)) then
                                is_top, is_bottom, i_right, i_left, i_any, identified_r = tdcf.check_array(data_r, DEFAULT_TD_LEVEL )
                                is_top, is_bottom, i_right, i_left, i_any, identified_l = tdcf.check_array(data_l, DEFAULT_TD_LEVEL )
                                if ((identified_l == true) and (identified_r == true)) then
                                    grip_ok = true
                                end
                            else
                                if data_try_count == DEFAULT_TD_IDENTIFICATION_LIMIT-1 then
                                    retry = true
                                end
                            end
                            data_try_count = data_try_count + 1
                        end
                        if retry ~= true then
                            next_force = current_force + DEFAULT_FORCE_INCREASE_STEP
                        end
                    end
                end
                if (DEBUG_OUTPUT == true) or (INFO_OUTPUT == true) then     
                    print("----LEFT----                            ----RIGHT----")
                    print("identified:")
                    print("  ",identified_l,"                           ",identified_r )
                    tdcf.print_arr(tdcf.data_arr_l() , tdcf.data_arr_r())
                    printf("Gripping - Force: %.2f \n", current_force)
                    print("Gripping - OK : ", grip_ok, "\n")
                end
                -- return results and inform about end of controlled gripping routine
                cmd.send(GRIP_CONTROL_CMD, identified, grip_ok)
                -- --> send: \aa\aa\aa\grip_cmd\02\00\ident\grip_ok\crc\crc
                
            end
            -- release / move open / conditional release
            if id == GRIP_RELEASE_CMD then
                if DEBUG_OUTPUT == true then
                    print("...RELEASING \n")
                end
                -- check grasping or clamping state
                if (object_clamped == true) then
                    if DEFAULT_COND_REL_ACTIVE == true then
                        -- implementation: specific edge node must be pressed on one of either arrays
                        local free_trigger = false
                        while (free_trigger ~= true) do
                            
                            tdcf.get_data_buffered()
                            local data_left = tdcf.data_arr_l()
                            local data_right = tdcf.data_arr_r()
                            if DEBUG_OUTPUT == true then
                                print("----LEFT----                            ----RIGHT----")
                                tdcf.print_arr(tdcf.data_arr_l(), tdcf.data_arr_r())
                            end
                            if ((data_left[1] ~= -1) and (data_right[1] ~= -1)) then
                                if ((data_left[6][6] >= DEFAULT_TD_LEVEL) or (data_right[6][1] >= DEFAULT_TD_LEVEL)) then
                                    free_trigger = true;
                                else
                                    sleep(300)
                                end
                            else
                                sleep(300)
                            end
                        end
                    end
                    -- stop applying force and move to default open
                    grasping.stop_clamping()
                    mc.move(DEFAULT_START_POSITION, DEFAULT_SPEED, 0x001)
                else
                    mc.move(DEFAULT_START_POSITION, DEFAULT_SPEED, 0x001)
                end
                -- -- return results and inform about end of conditional release routine
                cmd.send(GRIP_RELEASE_CMD, 1, 0);
                -- --> send: \aa\aa\aa\grip_rel\02\00\01\00\crc\crc
                object_clamped = false               
            end
            -- publish data from both sensors to host/link-device
            if id == PUB_DATA_CMD then
                if DEBUG_OUTPUT == true then
                    print("...DATA PUBLISHING \n")
                end
                -- read sensor data and generate single array with all values (val_L -> val_R)
                tdcf.get_data_buffered()
                tdcf.generate_data_out()

                if DEBUG_OUTPUT == true then
                    print("----LEFT----                            ----RIGHT----")
                    tdcf.print_arr(tdcf.data_arr_l(), tdcf.data_arr_r())
                end
                -- return results aka publish data
                cmd.send(PUB_DATA_CMD, tdcf.data_out())
            end
        end
    end
else
    print("NO CLIENT CONNECTED and Script set to ALWAYS_RUNNING = False")
end