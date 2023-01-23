% Brandon Medellin 
% Context: Data for train track experiment for Ali Hassani
% Visualization for the IMU, OptiTrack, and LiDAR data
% Variable names and formats are the same for backwards compatibility

clear
clc
%{ 
Extracted Variables
    Cyl     : [matrix] position of landmarks indicated by a 6 by 3 matrix where columns
              correspond to [X Y Z] coordinates
    LiDARpc : [cell array] containing LiDAR point cloud 
    riv     : [matrix] IMU pose over time where columns correspond to [Time _ Z Y X Z Y X]
              where the first cartesian is translational acceleration and second
              angular
    VICONpc : [matrix] OptiTrack pose over time where columns correspond to [Time 
              Pitch Yaw Roll X Y Z]
%}
load('Experimental_Data_09_12_22.mat')


%riv(1,1) - VICONpc(1,1)

%%
% figure(1)
% plot(riv(:,1),riv(:,3))
% hold on
% plot(riv(:,1),riv(:,4))
% plot(riv(:,1),riv(:,5))
% hold off

%%
% figure(2)
% plot(riv(:,1),riv(:,6),'r')
% hold on
% plot(riv(:,1),riv(:,7),'b')
% plot(riv(:,1),riv(:,8),'k')

%%
%bag = rosbag('Train_Data.bag');
% Opti = select(bag,'Topic','/vrpn_client_node/Train/pose');
%IMU = select(bag,'Topic','/RAWIMUA_message');
%messages = readMessages(IMU, 'DataFormat', 'struct');

Time = VICONpc(1:50:end,1);
OptiPsi = VICONpc(1:50:end,3);
dt = diff(Time);
dpsi = diff(OptiPsi);
dpsidt = dpsi./dt;
dpsidtdeg = rad2deg(dpsidt);

figure(3)
plot(riv(:,1)-riv(1,1),riv(:,6),'r'), hold on
plot(Time(1:end-1)-riv(1,1),dpsidtdeg,'k--',LineWidth=1.25)
% plot(bag.StartTime,20,'.b',LineWidth=10)
% plot(bag.EndTime,20,'.b',LineWidth=10)
%plot(messages{1,1}.Header.Stamp.Sec,20,'.b',LineWidth=10)
%ylim([-60,60])
hold off

%%
%riv(:,1) = riv(:,1) - 10;
% 
% 
% N = length(LiDARpc);
% Time = zeros(N,1);
% IMUPsi = zeros(N,1);
% OptiPsi = zeros(N,1);
% for i=1:N
%     Time(i) = mean(LiDARpc{i,1}(:,1));
%     [val,index] = min(abs(VICONpc(:,1)-Time(i)));
%     OptiPsi(i) = VICONpc(index,3);
%     
%     [val,index] = min(abs(riv(:,1)-Time(i)));
%     IMUPsi(i) = riv(index,6);
% end
% 
% dt = diff(Time);
% dpsi = diff(OptiPsi);
% dpsidt = dpsi./dt;
% 
% figure(4)
% plot(Time(1:end-1,1)-Time(1),rad2deg(dpsidt),'k')
% hold on
% plot(Time-Time(1),IMUPsi,'r')
% hold off


