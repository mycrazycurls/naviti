%% NAVITI_SensorsPP
%%
% NAViTi IMU, OptiTrack, and LiDAR Post-Processing Script
% Author: Brandon Medellin
%
% Last Update: 07/26/2022
%
% Post Processing of IMU, OptiTrack, and LiDAR data.
% Relevant variables are saved into a .mat file that can be used in another
% MATLAB script for analysis.
%
% See NAVITILabDocumentation.pdf for more info on the helper functions

clear
clc
format long

%%
% Define constants here. Constants are kept at the top level so they can be
% easily found. They are typically hardcoded values that the user may
% change to their needs.
const.bagfn = 'Train_Data.bag';
const.imutopic = '/RAWIMUA_message';
const.optitopic = '/vrpn_client_node/Train/pose';
const.vlptopic = '/velodyne_points';
const.landmarkcsv = 'landmark124635_10_31_22.csv';
const.expfn = 'Experimental_Data_10_31_22.mat';
const.expversion = '-v7.3';

% Check if bag file exists
if ~isfile(const.bagfn)
    error([const.bagfn ': Missing this bag file. Note: This code' ...
        'is currently set to use a single bag containing all topics.\n'])
end

% Check if landmark csv file exists
if ~isfile(const.landmarkcsv)
    error([const.landmarkcsv ': Missing this landmark csv file.' ... 
        ' Change name of constant or place file in folder.'])
end

% Read the ROS bag file into a MATLAB 'bag' object
bag = rosbag(const.bagfn);

%% 
% Post-Process IMU messages
IMU.bag = select(bag,'Topic',const.imutopic);
% IMU.processed = IMUPP(IMU.bag);
IMU.processed = IMUPP_BINARY(IMU.bag);

%% 
% Post-Process OptiTrack messages
Opti.bag = select(bag,'Topic',const.optitopic);
Opti.processed = OptiTrackPP(Opti.bag);

%% 
% Post-Process OptiTrack Landmark Positions
landmarks.options = delimitedTextImportOptions('Delimiter',{','});
landmarks.rawtable = readtable(const.landmarkcsv,landmarks.options);
landmarks.processed = LandmarksPP(landmarks.rawtable);

%%
% Post-Process LiDAR messages
LiDAR.bag = select(bag,'Topic',const.vlptopic);
LiDAR.processed = LiDARPP(LiDAR.bag);

%% Post Processing LiDAR
% Import Optitrack data from converted rosbag file
num_message = LiDAR.bag.NumMessages;

LiDARpc_out = {};
for i = 1:num_message
%     disp(i)
    LiDAR_data = readMessages(LiDAR.bag,i);
%     LiDAR_message = readMessages(select_topic_velodyne, 'DataFormat', 'struct');
    LiDARpci = LiDAR_data{1};
    LiDARpc = readXYZ(LiDARpci);
%     LiDARpc = rosReadXYZ(LiDARpci);
    LiDAR_timestamp = LiDAR.bag.MessageList(i,1);
    LiDAR_timestamp = LiDAR_timestamp{1,1};
    LiDARpci_intensity = readField(LiDARpci, 'intensity');
    
    % Output LiDAR data
    % Timestamp | azimuth elevation range intensity | X Y Z Position |
    LiDAR_PP = NaN*ones(size(LiDARpc,1),8);
    LiDAR_PP(:,1) = LiDAR_timestamp*ones(size(LiDARpc,1),1);
    LiDAR_PP(:,2) = [];
    LiDAR_PP(:,3) = [];
    LiDAR_PP(:,4) = [];
    LiDAR_PP(:,5) = LiDARpci_intensity;
    LiDAR_PP(:,6:8) = LiDARpc;
    
    LiDARpc_out{i} = LiDAR_PP;
    
%     clear LiDAR_data LiDAR_message LiDARpci LiDARpc LiDAR_timestamp LiDAR_PP
    clear LiDAR_data LiDARpci LiDARpc LiDAR_timestamp LiDAR_PP
end
LiDARpc_out = LiDARpc_out';

%% 
% Define export variables
export.riv = IMU.processed;
export.VICONpc = Opti.processed;
%export.LiDARpc = LiDAR.processed;
export.LiDARpc = LiDARpc_out;
export.Cyl = landmarks.processed;

%% 
% Save variables in the 'export' structure to a .mat file
save(const.expfn,'-struct','export',const.expversion);




