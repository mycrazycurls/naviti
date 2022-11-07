%% OptiTrackPP
%%
% ROS conveniently stores OptiTrack timestamped pos data into a struct
% already so no extra parsing on this end is needed. This function reads
% through the messages and stores them into a matrix. 
%
% * |_<https://www.mathworks.com/help/ros/ref/bagselection.html
% ros.BagSelection> *bag*_|: a bag containing messages under the desired
% topic (OptiTrack).
%
% * |_Nx7 matrix_ *pose*|:
% Formatted as such |[times pitch yaw roll x y z]|. Where |N| is the
% number of messages in the bag.

function pose = OptiTrackPP(bag)
    %%
    % Read the bag
    messages = readMessages(bag, 'DataFormat', 'struct');
    
    %% 
    % Pre-allocate a matrix
    N = bag.NumMessages;
    pose = zeros(N,7);
    
    %%
    % For each message store the position and orientation
    for i = 1:N
        pose(i,1) = messages{i,1}.Pose.Orientation.X;
        pose(i,2) = messages{i,1}.Pose.Orientation.Y;
        pose(i,3) = messages{i,1}.Pose.Orientation.Z;
        pose(i,4) = messages{i,1}.Pose.Orientation.W;
        pose(i,5) = messages{i,1}.Pose.Position.X;
        pose(i,6) = messages{i,1}.Pose.Position.Y;
        pose(i,7) = messages{i,1}.Pose.Position.Z;
    end
    
    %%
    % Replace columns with timestamps and calculated Euler angles
    [yaw,pitch,roll] = ali_quat2euler(pose(:,4), pose(:,2), pose(:,1), pose(:,3));
    timestamps = bag.MessageList{:,1};
    pose(:,1:4) = [timestamps pitch yaw roll];
  
end