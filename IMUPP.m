%% IMUPP
%%
% IMU timestamped messages are currently stored by ROS as a comma delimited
% string and this function parses each message to get desired values which
% include the acceleration and angular acceleration.
%
% * |_<https://www.mathworks.com/help/ros/ref/bagselection.html ros.BagSelection> *bag*_|:
% a bag containing messages under the desired topic (IMU). 
%
% * |*options*|:
% optional parameters in case the format of incoming data changes
% 
% * |_uint32_ *numfields*|:
% the expected number of fields after splitting by comma
%
% * |_uint32_ *accel_index*|:
% the expected indices of the fields corresponding to the acceleration
%
% * |_unint32_ *gyro_index*|:
% the expected indices of the fields corresponding to the angular acceleration
%
% * |_double_ *accel_comp*|:
% the acceleration scale factor
%
% * |_double_ *ang_comp*|:
% the angular acceleration scale factor
%
% * |_Nx8 matrix_ *pose*|:
% Formatted as such |[time NaN accel(Z Y X) ang(Z Y X)]| . Where |N| is the number of
% messages in the bag.

function pose = IMUPP(bag, options)

    %%
    % Define arguments and defaults
    arguments
        bag ros.BagSelection
        options.num_fields (1,1) uint32 = 20
        options.accel_index (1,3) uint32 = 15:17
        options.gyro_index (1,3) uint32 = 18:20
        options.accel_comp (1,1) double = 200/(2^31)*200
        options.ang_comp (1,1) double = 720/(2^31)*200
    end
    
    %%
    % Read the bag
    messages = readMessages(bag, 'DataFormat', 'struct');
    
    %%
    % Pre-allocate vectors
    N = bag.NumMessages;
    accel_list = NaN(N,3); 
    gyro_list = NaN(N,3);
    
    %% 
    % For each message, get the desired fields and store into a vector
    for i=1:N
        data = messages{i,1}.Data;
        data = split(data, '*');
        data = str2double(split(cell2mat(data(1)), ','));

        if size(data,1) == options.num_fields
            accel = data(options.accel_index)*options.accel_comp;
            accel_list(i,:) = accel;
            gyro = data(options.gyro_index)*options.ang_comp;
            gyro_list(i,:) = gyro;
        end
    end
    
    %%
    % Create matrix and remove any missing values
    timestamps = bag.MessageList{:,1};
    pose = rmmissing([timestamps zeros(N,1) accel_list gyro_list]);
    pose(:,2) = NaN(length(pose),1);
end

%% Example
%  Say the index of the angular acceleration measurements are expected at field
%  indices 20:22. Then use:
%
%   IMUPP(bag, 'gyro_index', 20:22)
%   IMUPP(bag, gyro_index=20:22)