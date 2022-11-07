%% LiDARPP
%%
% Parses the LiDAR messages from a ROS bag.
% 
% * |_ros.BagSelection_ *bag*|:
% a bag containing only messages under the desired topic (LiDAR). 
%
% * |*options*|:
% optional parameters to adjust desired output
%
% * |_char_ *legacy*|:
% specifies whether to rollback to an older output format. 'ali':
% contains some empty arrays for the azimuth elevation and range.
% 'bran': excludes the empty arrays.
%
% * |_Nx1 cell array_ *pclouds*|:
% N is the number of messages. Each cell contains a matrix with pc data.

function pclouds = LiDARPP(bag, options)

    %%
    % Define arguments and defaults
    arguments
        bag ros.BagSelection
        options.legacy char = 'ali'
    end

    %%
    % Read the bag. The messages are read as a PointCloud2 object.
    messages = readMessages(bag);
    
    %%
    % Pre-allocate a cell-array to store N matrices of point cloud data
    N = bag.NumMessages;
    pclouds = {N};
    timestamps = bag.MessageList{:,1};

    %%
    % For each message, get the desired values
    for i = 1:N
        pcobject = messages{i,1};
        intensity = readField(pcobject, 'intensity');
        pc = readXYZ(pcobject);
        L = length(pc);
        times = timestamps(i)*ones(L,1);
        azimuth = NaN(L,1);
        elevation = NaN(L,1);
        range = NaN(L,1);

        %%
        % An option that acts more so as an example of how a rollback to a
        % previous format could be implemented so that older codes could
        % still be operable. This switches the format of the returned data.
        switch options.legacy
            case 'ali'
                pclouds{i,1} = [times azimuth elevation range intensity pc];    
            case 'bran'
                pclouds{i,1} = [times intensity pc];    
        end

    end

end