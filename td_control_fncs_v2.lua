--- td control fncs v2 module
-- @author Sascha Teutoburg-Weiss
-- @usage implements functions to read and manipulate TD sensor data, namely for td_grip_control_v2 application
-- @name td_control_fncs_v2
-- version v2
-- 2023-12-15

local NAME = ...    -- require name search capture
local M = { }       -- module functions table

-- TD sensor variables
local data_arr_l_ = {{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}}
local data_arr_r_ = {{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}}
local data_out_ = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}

--- Getter function for left configured data array
-- @return 2-dimensional table of 6x6 array
function M.data_arr_l()
    return data_arr_l_
end
--- Getter function for right configured data array
-- @return 2-dimensional table of 6x6 array
function M.data_arr_r()
    return data_arr_r_
end
--- Getter function for output_data in single instance
-- @return 1-dimensional table of 1x72 array (36(L) + 36(R))
function M.data_out()
    return data_out_
end

--- Print given array(s) representation in console
-- 1-D    :  [a,b,c,d] -> "a \n b \n c \n d \n"
-- 2-D    :  [ [a,a,a] [b,b,b] [c,c,c] ] ->  "a a a \n b b b \n c c c \n"
-- 2x2-D  :  [ [a,b] [c,d] ] + [ [e,f] [g,h] ] -> "a b   e f \n c d   g h \n"
-- @param arr First 1-D or 2-D array to print
-- @param arr2 Second 2-D array to print (only same dimensions as first)
function M.print_arr(arr, arr2)
    local debug_string = ""
    for i=1,#arr do
        if (type(arr[i]) == "table") then
            for j=1,#arr[i] do
                local raw = arr[i][j];
                -- 4 digits per node
                debug_string = debug_string..string.format("%04d",raw)
                -- least possible format based on value
                --debug_string = debug_string..tostring(raw);
                if j ~= #arr[i] then
                    debug_string = debug_string.."  ";
                end
            end
            if (type(arr2) == "table") then
                if (type(arr2[i]) == "table") then
                    debug_string = debug_string.."      ";
                    for k=1,#arr2[i] do
                        local raw2 = arr2[i][k];
                        debug_string = debug_string..string.format("%04d",raw2)
                        --debug_string = debug_string..tostring(raw2)
                        if k ~= #arr2[i] then
                            debug_string = debug_string.."  ";
                        end
                    end
                end
            end
        else
            debug_string = debug_string..tostring(arr[i])
        end
        debug_string = debug_string.."\n"
    end
    print(debug_string)
end

--- Combines data bytes read from TD to 2-D sensor-data array
-- @param data_table 1-dimensional byte table
-- @param n_byte_p_val Number of bytes contributing to each value
-- @param t_row Number of rows provided
-- @param t_col Number of coloumns provided
-- @param endian Little/Big-Endian byte format [0: Little Endian | 1: Big Endian]
-- @return 2-dimensional table with t_row*t_col 
function M.combine_data_to_array(data_table,n_byte_p_val,endian_format,t_row, t_col)
    local data_byte_comb = {}
    local n_r_arr = t_row
    local n_c_arr = t_col
    local n_byte_p = n_byte_p_val
    for i=1,#data_table/(n_r_arr*n_byte_p) do
        local id = (n_r_arr*n_byte_p) * i - (n_r_arr*n_byte_p - 1)
        for j=1,#data_table/(n_c_arr*n_byte_p)  do
            local id_2 = id + n_byte_p*j - n_byte_p
            if data_byte_comb[j] == nil then
                data_byte_comb[j] = {}
            end
            if n_byte_p_val > 1 then
                if endian_format == 0 then
                    -- Little Endian
                    data_byte_comb[j][i] = (data_table[id_2+1]*2^8) + data_table[id_2]
                    --data_byte_comb[j][i] = math.floor(((data_table[id_2+1]*2^8 + data_table[id_2])/4096)*250)
                elseif endian_format == 1 then
                    -- Big Endian
                    data_byte_comb[i][j] = data_table[id_2]*2^8 + data_table[id_2+1]
                end
            else
                data_byte_comb[i][j] = data_table[id_2]
            end
            
            
        end
    end
    return data_byte_comb
end

--- Flip array arr in X or Y Orientation
-- ONLY use one flip per function call or Y will overwrite X
-- @param arr table which should be flipped
-- @param in_x boolean for flip in x orientation (coloumn switch)
-- @param in_y boolean for flip in y orientation (coloumn switch)
-- @return dimensional preserved flipped array
function M.flip_array(arr, in_x, in_y)
    local flipped = {}
    if (in_x == true) then
        for i=1,#arr do
            if flipped[i] == nil then
                flipped[i] = {}
            end
            for j=1,#arr[i] do
                flipped[i][j] = arr[i][(#arr[i])+1-j]
            end
        end
    end
    if (in_y == true) then
        for i=1,#arr do
            flipped[i] = arr[#arr[i]+1-i]
        end
    end
    return flipped
end

--- Rotate array in specified direction
-- @param arr table which should be rotated
-- @param dir rotate 1:clockwise | -1:counter-clockwise
-- @return dimensional preserved rotated array
function M.rotate_array(arr, dir)
    local rotated = {}
    -- initialize new array
    for i=1, #arr do
        for j=1, #arr[i] do
            if rotated[j] == nil then
                rotated[j] = {}
            end
            rotated[j][i] = {}
        end
    end
    -- fill data
    for i=1, #arr do
        for j=1, #arr[i] do
            if (dir == 1) then
                rotated[#arr[i]+1-j][i] = arr[i][j]
            else
                rotated[j][#arr+1-i] = arr[i][j]
            end
        end
    end
    return rotated
end

--- Send GET_DATA_CMD to finger and catch reply
-- multiple based on TIMEOUT_TD_TRY, TIMEOUT_TD_MS and TIMEOUT_TD_DIVIDER
-- @param finger_index Integer-Index of finger to communicate
-- @return complete data_frame of TD sensor (HDLC frame) or {-1} when timed-out/not-possible
function M.get_finger_data(finger_index)
    local f_indx = finger_index
    local timeout_count = 0
    local try_count = 0
    local bytes_available = 0
    local read_valid = 0

    local reply = {-1}
    
    while((read_valid ~= 1) and (try_count < TIMEOUT_TD_TRY)) do
        try_count = try_count + 1
        finger.write(f_indx, READ_DATA_COMMAND)
        --sleep(TIMEOUT_TD_DIVIDER)
        
        while((timeout_count ~= TIMEOUT_TD_MS*TIMEOUT_TD_DIVIDER) and (bytes_available ~= ACK_PACKAGE_SIZE)) do
            bytes_available = finger.bytes_available(f_indx)
            timeout_count = timeout_count + TIMEOUT_TD_DIVIDER
            sleep(TIMEOUT_TD_DIVIDER);
        end

        if timeout_count == (TIMEOUT_TD_MS*TIMEOUT_TD_DIVIDER) then
            if bytes_available ~= 0 then
                finger.read(f_indx)
            end
            reply = {-1}
            printf("!! TD-CMD-TIMEOUT on finger: %d", f_indx)
        else
            reply = finger.read(f_indx)
        end
        if ((type(reply) == "table") and (reply[1] == 126) and (reply[3] == 18) and (#reply == 84)) then
            read_valid = 1
            finger.write(f_indx, MASTER_ACK_COMMAND)
        else
            --finger.write(f_indx, MASTER_ACK_COMMAND)
            printf("!!! READ ERROR on TD finger: %d \n", f_indx)
            reply = {-1}
            timeout_count = 0
            while(timeout_count <= 160) do
                timeout_count = timeout_count + 10
                sleep(10);
            end
        end
    end
    return reply
end

--- Aqcuire complete data_frame from both fingers
-- strips down to data only part
-- combines data bytes to array representation
-- flip accordingly to match oriantation in gripping system
-- stores data tables in data_arr_x_ variables
function M.get_data_buffered()
    local data_tab_frame = {}   -- HDLC_data_frame buffer table
    local data_tab_buffer = {}  -- Pure_data buffer table
    
    for f_indx = 0, (finger.count() - 1) do
        data_tab_frame = M.get_finger_data(f_indx)
        if (data_tab_frame[1] ~= -1) then
            -- !!! STRIPS AWAY HDLC FRAME
            for i=4,75 do
                data_tab_buffer[i-3] = data_tab_frame[i]
            end
            if ((f_indx == FINGER_LEFT) and (data_tab_buffer ~= nil)) then
                data_arr_l_ = M.combine_data_to_array(data_tab_buffer,2,0,6,6)
                -- new fingers normal
                data_arr_l_ = M.flip_array(data_arr_l_, false, true)
                data_arr_l_ = M.flip_array(data_arr_l_, true, false)
            elseif ((f_indx == FINGER_RIGHT) and (data_tab_buffer ~= nil)) then
                data_arr_r_ = M.combine_data_to_array(data_tab_buffer,2,0,6,6)
                -- new fingers normal
                data_arr_r_ = M.flip_array(data_arr_r_, false, true)
            end
        else
            if f_indx == FINGER_LEFT then
                data_arr_l_ = {-1}
            elseif f_indx == FINGER_RIGHT then
                data_arr_r_ = {-1}
            end
        end
    end
end

-- Checks if array has more specified number of data points over specified level 
-- @param arr The array which should be checked
-- @param val The value which is evaluated as level/threshold
-- @param aim The aim of points exxeding the level
-- @param p = true: print | false or nil: no print to console
-- @return The number counted of points exeding level + boolean result
function M.check_array_value(arr,val,aim,p)
    if p == nil or p == true then
        print("checking for array.....value")
        M.print_arr(arr)
        p = false
        local p_init = true
    end
    local rslt, rslt_v = false, false
    local cnt, cnt_v = 0, 0
    for k, v in pairs(arr) do
        if type(v) == "number" then
            if v >= val then
                cnt = cnt + 1
            end
        elseif type(v) == "table" then
            cnt_v, rslt_v = M.check_array_value(v,val,aim,p)
            cnt = cnt + cnt_v
            rslt = rslt_v or rslt
        end
    end
    if (cnt >= aim) then
        rslt = true
    end
    return cnt, rslt
end

--- Checks complete 6x6 array/sub-arrays for specified conditions
-- @param arr The array which should be checked
-- @param level The level/threshold for evaluating sensor node as active
-- @return boolean upper/lower/left/right/anyz/ok conditions
function M.check_array(arr, level)
    if DEBUG_OUTPUT == true then
        print("checking for array.....")
        M.print_arr(arr)
    end
    local is_up, is_down, is_left, is_right, is_any, is_ok =
        false, false, false, false, false, false
    local is_ok_2, is_ok_4, is_ok_6 = false, false,false
    local rslt = false

    local slice_upper = {}      -- first row
    local slice_lower ={}       -- last row
    local slice_left = {}       -- first coloumn
    local slice_right = {}      -- last coloum
    local slice_middle_2 = {}   -- middle 2x2 region
    local slice_middle_4 = {}   -- middle 4x4 region
    
    local slice_m_2_cnt = 1
    local slice_m_4_cnt = 1
    for i=1,#arr do
        slice_left[i]  = arr[i][1]
        slice_right[i] = arr[i][#arr[i]]
        for j=1, #arr[i] do
            if i == 1 then
                slice_upper[j] = arr[i][j]
            elseif i == #arr then
                slice_lower[j] = arr[i][j]
            else
                if j > 1 and j < #arr[i] then         
                    slice_middle_4[slice_m_4_cnt] = arr[i][j]
                    slice_m_4_cnt = slice_m_4_cnt + 1
                    if i > 2 and i < 5 then
                        if j > 2 and j < 5 then
                            slice_middle_2[slice_m_2_cnt] = arr[i][j]
                            slice_m_2_cnt = slice_m_2_cnt + 1
                        end
                    end
                end            
            end
        end
    end
    local t = 0 -- counter reply of function calls coming up
    t, is_up    = M.check_array_value(slice_upper,level,1,false)
    t, is_down  = M.check_array_value(slice_lower,level,1,false)
    t, is_left  = M.check_array_value(slice_left,level,1,false)
    t, is_right = M.check_array_value(slice_right,level,1,false)
    t, is_ok_2  = M.check_array_value(slice_middle_2,level,1,false)
    t, is_ok_4  = M.check_array_value(slice_middle_4,level,2,false)
    t, is_ok_6  = M.check_array_value(arr,level,3,false)
    if t >= 1 then
        -- last t-counter in 6x6 array check function called
        is_any = true
    end
    is_ok = is_ok_2 or is_ok_4 or is_ok_6

    return is_up, is_down, is_left, is_right, is_any, is_ok
end

--- Generate complete data field of both sensors/fingers
-- takes data_arr_x_ variables in current state for processing
-- updates data_out_ variable for further usage
-- overrides eventuall error/false readings with all 0s in data_arr_x_ variables
function M.generate_data_out()
    if (data_arr_l_[1] == -1) then
        data_arr_l_ = {{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}}
    end
    if (data_arr_r_[1] == -1) then
        data_arr_r_ = {{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}}
    end
    for i=1, 6 do
        for j=1, 6 do
            if ((data_arr_l_ ~= nil) and (data_arr_l_[i] ~= nil) and (data_arr_l_[i][j] ~= nil)) then
                if data_arr_l_[i][j] >= DEFAULT_TD_LEVEL then
                    data_out_[(i-1)*6+j] = 1
                else
                    data_out_[(i-1)*6+j] = 0
                end
            end
            if ((data_arr_r_ ~= nil) and (data_arr_r_[i] ~= nil) and (data_arr_r_[i][j] ~= nil)) then
                if data_arr_r_[i][j] >= DEFAULT_TD_LEVEL then
                    data_out_[(i-1)*6+36+j] = 1
                else
                    data_out_[(i-1)*6+36+j] = 0
                end
            end
        end
    end
end

--- returns function table for module require calls
return M