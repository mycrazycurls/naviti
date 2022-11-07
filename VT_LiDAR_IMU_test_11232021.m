% Active Animation for LiDAR
close all
clear
clc
load('Experimental_Data_09_12_22.mat')

for ii=1:length(LiDARpc)
LiDARpc{ii,1}(:,6:8)=LiDARpc{ii,1}(:,6:8)*1e3;
end

% fixing the cylinder position (x=opposit of door - z= upward - y=closet)
Cyl([3 6],:)= Cyl([6 3],:);
Cyl([4 6],:)= Cyl([6 4],:); 

% fixing the OptiTrack(Nav frame) (x=opposit of door - z= upward - y=closet)
VICONpc(:,5:7) = VICONpc(:,5:7)*1e3;
% VICONpc(:,5:7)= ([0 1 0;-1 0 0;0 0 1]*[1 0 0;0 0 -1;0 1 0]*VICONpc(:,5:7)')';
% VICONpc(:,2:4)= ([0 1 0;-1 0 0;0 0 1]*[1 0 0;0 0 -1;0 1 0]*VICONpc(:,2:4)')';

CylBiasRemove_flag=1;
if CylBiasRemove_flag
Cyl(1,1)=-0.928564;            Cyl(1,3)=-3.433254;
Cyl(2,1)=-0.923672;            Cyl(2,3)=-1.916975;
Cyl(3,1)=1.034743;             Cyl(3,3)=-1.888620;
Cyl(4,1)=-2.193204;            Cyl(4,3)=1.978909;
Cyl(5,1)=-0.491540 ;           Cyl(5,3)=3.499557 ;
Cyl(6,1)=1.876086;             Cyl(6,3)=2.076122;
Cyl(:,[1 3]) =Cyl(:,[1 3])*1e3;
end

Cyl_Intensity_flag=0; % This section define cylinders' expected intensity 
% Cyl_In(1,1)=190;             Cyl_In(1,2)=220;
% Cyl_In(2,1)=0;               Cyl_In(2,2)=8;
% Cyl_In(3,1)=50;              Cyl_In(3,2)=70;
% Cyl_In(4,1)=0;               Cyl_In(4,2)=8;
% Cyl_In(5,1)=0;               Cyl_In(5,2)=8;
% Cyl_In(6,1)=50;              Cyl_In(6,2)=70;

% frm_len(:,1:12)=[];
% LiDARpc(1:99949,:)=[];
% LiDARpc = LiDARpc
flag_navPlot = 0;
flag_3Dplot = 1;
flag_FEplot = 0;
falg_polyfitremove = 0;

% Segmentation parameters
rgeLim = 5000;
% hgtMin = 100;
% hgtMax = 1500;
% Feature extraction parameters
minPtsPerObj = 20;%10;
minPt2PtDist = 700;
maxNbObj = 15;
% Display parameters
thetVec = 0:2*pi/50:2*pi;

if flag_3Dplot == 1
    figure(2)
    hold on
    grid off
    view(-91,67);
    xlim([-6000 6000]);
    ylim([-10000 10000]);
    zlim([0 4000]);
    set(gcf,'position',[515 650 500 400]);
    if flag_navPlot
        figure(3), hold on, grid off
        view(-90,90); xlim([-4000 5000]); ylim([-4500 4500]); zlim([0 4000]);        
        set(gcf,'position',[1015 650 500 400])
    end
end
nbCyl = size(Cyl,1);
jmin = 1;
% nbInc = length(frm_len);
nbInc = 700; %length(LiDARpc);
cylCenterXlasVec = zeros(nbInc ,maxNbObj);
cylCenterYlasVec = zeros(nbInc ,maxNbObj);
cylRadVec = zeros(nbInc ,maxNbObj);
InVec = zeros(nbInc ,maxNbObj);
timeVec = zeros(nbInc,1);
LiDARextr = zeros(nbInc , 2*maxNbObj);
LiDARasso = zeros(nbInc , 3*nbCyl);
LiDARassoNav = zeros(nbInc , 3*nbCyl);
LiDARassoError = zeros(nbInc , nbCyl);
nbObjVec = zeros(1,nbInc);
xcoMin = -2350; xcoMax = 2050;
ycoMin = 400; ycoMax = 2000;
zcoMin = -3750; zcoMax = 3800;
%%%% IMU Figure
IMUangVec = zeros(nbInc,3);
VICONpcSyncTV = zeros(nbInc,6);
IMU_SyncTV = zeros(nbInc,6);
jiterIMU=1;


for jind = 1:nbInc
    timeVec(jind) = mean(LiDARpc{jind,1}(:,1));
    [val,indLiVImatch] = min(abs(VICONpc(:,1)-timeVec(jind)));
    ttheta   = VICONpc(indLiVImatch,2)+ deg2rad(0);
    psi      = VICONpc(indLiVImatch,3);
    pphi     = VICONpc(indLiVImatch,4);

    pphi_store(jind)   = pphi;
    ttheta_store(jind) = ttheta;
    psi_store(jind)    = psi;
    
    if jind ~= 1
        timediff(jind-1) = (timeVec(jind)-timeVec(jind-1));
        pphi_rate(jind-1)= (pphi_store(jind)-pphi_store(jind-1))/timediff(jind-1);
        ttheta_rate(jind-1)= (ttheta_store(jind)-ttheta_store(jind-1))/timediff(jind-1);
        psi_rate(jind-1)= (psi_store(jind)-psi_store(jind-1))/timediff(jind-1);
    end
       
tMat = [cos(psi) 0 -sin(psi);0 1 0;sin(psi) 0 cos(psi)]*[cos(ttheta) sin(ttheta) 0;-sin(ttheta) cos(ttheta) 0;0 0 1]*[cos(pphi) 0 -sin(pphi);0 1 0;sin(pphi) 0 cos(pphi)]; % Good

% from navigation to body
rotOpti2Body = [cos(pphi) sin(pphi) 0;-sin(pphi) cos(pphi) 0;0 0 1]*[1 0 0;0 cos(ttheta) sin(ttheta);0 -sin(ttheta) cos(ttheta)]*[cos(psi) 0 -sin(psi);0 1 0;sin(psi) 0 cos(psi)];
rotBody2Opti = rotOpti2Body';
rotMat       = rotBody2Opti;
% from sensor to body frame
    rotSen2Body = [0 0 1;0 1 0;-1 0 0]*[1 0 0;0 0 1;0 -1 0];  % 90 degree around x and -90 degree around y

% Compensating calibration problems
    thet1 = ((0)*pi/180+(0*pi/2)); %pi/6;
%     rotMat1 = [cos(thet1) sin(thet1) 0;-sin(thet1) cos(thet1) 0;0 0 1];
rotMat1 = [cos(thet1) 0 -sin(thet1);0 1 0;sin(thet1) 0 cos(thet1)];
  
    trajVec = (VICONpc(:,5:7)')';
    trV    = VICONpc(indLiVImatch,5:7);
    LiDAR_Navtmp = (((rotMat1*rotMat*rotSen2Body*(LiDARpc{jind,1}(:,6:8))')'+repmat(trV',1,length(LiDARpc{jind,1}))')')';

    if flag_3Dplot
        figure(1)
        plot3(Cyl(:,1),Cyl(:,2),Cyl(:,3),'rs','MarkerFaceColor','r');
        view(-180,0)
        xlabel('x (mm)')
        ylabel('y (mm)')
        zlabel('z (mm)')
        xlim([-4000 5000]); ylim([-4500 4500]); zlim([-4500 4500]);    
        hold on
        grid off
        plot3(trV(1),trV(2),trV(3),'pb','MarkerFaceColor','b','Linewidth',2)
        plot3(LiDAR_Navtmp(:,1),LiDAR_Navtmp(:,2),LiDAR_Navtmp(:,3),'c.','Color',[1 1 1]*0.5,'Linewidth',1);%cmap(LiDARpc(j,6)+1,:));
        plot3(trajVec(:,1),trajVec(:,2),trajVec(:,3),'k-','Linewidth',2);%cmap(LiDARpc(j,6)+1,:));
        hold off
        axis square
        title(['jind increment: ' num2str(jind)])
        set(gcf,'position',[10 650 500 400])
        grid on
        F(jind)=getframe(gcf);
    end

    tmp = find(LiDAR_Navtmp(:,1)<xcoMax & LiDAR_Navtmp(:,1)>xcoMin &...
          LiDAR_Navtmp(:,2)<ycoMax & LiDAR_Navtmp(:,2)>ycoMin &...
          LiDAR_Navtmp(:,3)<zcoMax & LiDAR_Navtmp(:,3)>zcoMin);

    if flag_3Dplot
        figure(2)
        plot3(0,0,0,'kp','MarkerFaceColor','k', 'linewidth',1,'MarkerSize',6);
        xlim([-6 6]); ylim([-6 6]); zlim([-1 4]);
        hold on
        grid off         
        plot3(LiDARpc{jind,1}(:,6)/1e3,LiDARpc{jind,1}(:,7)/1e3,LiDARpc{jind,1}(:,8)/1e3,'c.','Color',[1 1 1]*0.5,'Linewidth',1);%cmap(LiDARpc(j,6)+1,:));
        hold off
        legend('LiDAR','LiDAR point cloud measueremnts','Location','SouthWest')
        legend BOXOFF
        xlabel('x (m)')
        ylabel('y (m)')
        zlabel('z (m)')
        grid on
        box off
        set(gcf,'position',[515 650 500 400])
        F1(jind)=getframe(gcf); 
        
        if flag_navPlot
            figure(3)
            scatter3(Cyl(:,1),Cyl(:,2),Cyl(:,3),'bs','MarkerFaceColor','k');
            view(-180,0); xlim([-4000 5000]); ylim([-5000 5000]); zlim([-5000 5000]);
            hold on
            plot3(LiDAR_Navtmp(:,1),LiDAR_Navtmp(:,2),LiDAR_Navtmp(:,3),'c.','Color',[1 1 1]*0.5,'Linewidth',1);%cmap(LiDARpc(1,6)+1,:));
            plot3(LiDAR_Navtmp(tmp,1),LiDAR_Navtmp(tmp,2),LiDAR_Navtmp(tmp,3),'ro','MarkerFaceColor','r','Linewidth',2);%cmap(LiDARpc(1,6)+1,:));
            plot3([xcoMin xcoMax xcoMax xcoMin xcoMin],[0 0 0 0 0],[zcoMin zcoMin zcoMax zcoMax zcoMin],'k-','Linewidth',2)
            hold off
            set(gcf,'position',[1015 650 500 400]);
        end
    end
%%    
%     if flag_FEplot
        %%% landmark extraction
        tmp1 = tmp;
        indObj = zeros(10000,maxNbObj);
        objNb = 1;
        
        % find points close to point index
        for jj=1:size(Cyl,1)
            indtmp = find(sqrt(sum([LiDAR_Navtmp(tmp,1)-Cyl(jj,1) LiDAR_Navtmp(tmp,3)-Cyl(jj,3)]'.^2))<minPt2PtDist);
            if ~isempty(indtmp)&& length(indtmp)>minPtsPerObj
                indObj(1:length(indtmp),objNb) = tmp(indtmp);
                objNb = objNb+1;
            end
        end
        
        indObj(sum(indObj')==0,:)=[];
        indObj(:,sum(indObj)==0)=[];
        nbObj = size(indObj,2);
        cylCenterCoord = zeros(2,nbObj);
        cylRad = zeros(1,nbObj);
        In = zeros(1,nbObj);
        for jj = 1:nbObj
        % data points to fit
           dataTmp = [LiDARpc{jind,1}(nonzeros(indObj(:,jj)),6) LiDARpc{jind,1}(nonzeros(indObj(:,jj)),7)];
            % initial guess
            r0 = 304.8/2;%160; 
            a0 = mean(LiDARpc{jind,1}(nonzeros(indObj(:,jj)),6)); b0 = mean(LiDARpc{jind,1}(nonzeros(indObj(:,jj)),7));
            In(jj) = mean(LiDARpc{jind,1}(nonzeros(indObj(:,jj)),5));
           
            [cenTmp, radTmp, l]= circleFitMJnew(dataTmp,r0,a0,b0,jind,jj);

            cylCenterCoord(:,jj) = cenTmp';
            cylRad(jj) = radTmp;

        end

    %%    
    
        cylRadMax = 700;
        cylRadMin = 50;
        tmpCyl = find(cylRad>cylRadMax | cylRad<cylRadMin);
        cylRad(tmpCyl) = [];
        cylCenterCoord(:,tmpCyl) = [];
        nbObj = length(cylRad);
        nbObj_test(jind)=nbObj;
      
        if flag_FEplot
            figure(4);
            plot(0,0,'kp','MarkerFaceColor','k', 'linewidth',1,'MarkerSize',6)
            view(-80,67); ylim([-6 6]); xlim([-6 6]); zlim([0 1]);
            hold on
            %         plot(LiDARpc(tmp,6)/1e3,LiDARpc(tmp,7)/1e3,'c.','Color',[1 0 0.3]*0.6)
            %           plot(LiDARpc{jind,1}(tmp,6)/1e3,LiDARpc{jind,1}(tmp,7)/1e3,'c.','Color',[1 0 0.3]*0.6)
            for jj = 1:nbObj
                colorc = [1 0 0];%*jj/nbObj;
                %             plot(LiDARpc(nonzeros(indObj(:,jj)),6)/1e3,LiDARpc(nonzeros(indObj(:,jj)),7)/1e3,'r.','Color',colorc )
                plot(LiDARpc{jind,1}(nonzeros(indObj(:,jj)),6)/1e3,LiDARpc{jind,1}(nonzeros(indObj(:,jj)),7)/1e3,'r.','Color',colorc )
                plot((cylRad(jj)*cos(thetVec)+cylCenterCoord(1,jj))/1e3,(cylRad(jj)*sin(thetVec)+cylCenterCoord(2,jj))/1e3,'b-', 'linewidth',0.5)
            end
            hold off
            %         legend('LiDAR','Segmented data','Extracted data','Fitted cylinders','Location','SouthWest')
            legend({'LiDAR','Extracted data','Fitted cylinders'},'Location','SouthWest','FontSize',12)
            legend BOXOFF
            xlabel('x (m)')
            ylabel('y (m)')
            zlabel('z (m)')
            %         box off
            %         set(gca,'XLim',[-1 1]*rgeLim/1e3,'YLim',[-1 1]*rgeLim/1e3)
            %         axis equal
            grid on
            set(gcf,'position',[1515 650 500 400])
            F2(jind)=getframe(gcf);
        end
         
        
        cylCenterXlasVec(jind,1:nbObj) = cylCenterCoord(1,:);
        cylCenterYlasVec(jind,1:nbObj) = cylCenterCoord(2,:);
        cylRadVec(jind,1:nbObj) = cylRad;
        nbObjVec(jind) = nbObj;

        if flag_FEplot
            figure(25)
            plot(0,0,'kp','MarkerFaceColor','k', 'linewidth',1,'MarkerSize',6)
            hold on
            for jj = 1:nbObj
                colorc=(LiDARpc{jind,1}(nonzeros(indObj(:,jj)),5));
                scatter3(LiDARpc{jind,1}(nonzeros(indObj(:,jj)),6)/1e3,LiDARpc{jind,1}(nonzeros(indObj(:,jj)),7)/1e3,LiDARpc{jind,1}(nonzeros(indObj(:,jj)),8)/1e3,1,colorc)%colorc(1:colsize,:))
            end

            hold off
            %         legend('LiDAR','3D Segmented data','3D Extracted data','Location','SouthWest')
            legend({'LiDAR','3D Extracted data'},'Location','SouthWest','FontSize',12)
            legend BOXOFF
            xlabel('East (m)')
            ylabel('North (m)')
            box off
            set(gca,'XLim',[-1 1]*rgeLim/1e3,'YLim',[-1 1]*rgeLim/1e3)
            set(gcf,'position',[10 200 500 400])
            axis equal


            figure(5)
            hold on
            plot(jind,nbObj, 'b.', 'linewidth',3)
            set(gca,'XLim',[0 1]*nbInc,'YLim',[0 1]*maxNbObj)
            set(gcf,'position',[10 -100 500 400])
        end

        LiDARassoNavtmp = zeros(nbCyl, 3);
        indAssoVec=zeros(nbInc,nbCyl);
        for jj = 1:nbObj        
            % preprocessed LiDAR data
            LiDARextr(jind,2*jj-1:2*jj) = cylCenterCoord(1:2,jj);
            % Associated LiDAR data
            cylPosNav_tmp = (rotMat1*rotMat*rotSen2Body*[cylCenterCoord(1:2,jj);0]+trV')';
            cylPosNav=cylPosNav_tmp(1,[1 3]);
            [val,indAsso] = min(sqrt(sum(abs(Cyl(:,[1 3])-repmat(cylPosNav,nbCyl,1)).^2, 2)));
            LiDARassoErrorVec=abs(Cyl(indAsso,[1 3])-(cylPosNav));
            LiDARRangeError= norm(Cyl(indAsso,[1 3]))-norm((cylPosNav));
            LiDARbearingError= atan2(Cyl(indAsso,3),Cyl(indAsso,1))-atan2(cylPosNav(1,2),cylPosNav(1,1));
            
            
            % Removing wrong extractions
                if abs(val)>5000
                    cylCenterCoord(1:2,jj)=zeros(1,2); In(jj)=0; cylPosNav=zeros(1,2);
                    val=0; LiDARassoErrorVec(1,1:2)=zeros(1,2); LiDARRangeError=0;
                end

            LiDARasso(jind,2*indAsso-1:2*indAsso) = cylCenterCoord(1:2,jj);
            LiDARasso(jind,indAsso+(2*nbCyl)) = In(jj);                       
            LiDARassoNav(jind,2*indAsso-1:2*indAsso) = cylPosNav;
            LiDARassoNav(jind,indAsso+(2*nbCyl)) = In(jj);
            LiDARassoNavtmp(indAsso,1:2) = cylPosNav;
            LiDARassoNavtmp(indAsso,3) = In(jj);
            
            LiDARassoError(jind,indAsso) = val;            
            LiDARassoError_X(jind,indAsso) = LiDARassoErrorVec(1,1);
            LiDARassoError_Y(jind,indAsso) = LiDARassoErrorVec(1,2);
            LiDARRangeErrorTotal(jind,indAsso) = LiDARRangeError;
            LiDARbearingErrorTotal(jind,indAsso) = LiDARbearingError;
           
            % LiDAR Intensity Measurments            
        end
        
            LiDAR_Intensity(jind,:)=LiDARassoNavtmp(:,3)';
    
if flag_FEplot
    figure(6)
    colorc = colormap('jet'); % jet
    colorc = resample(colorc, nbCyl, size(colorc, 1));
    colorc(colorc>1)=1;colorc(colorc<0)=0;
    hold on
    plot(Cyl(:,1),Cyl(:,3),'.','MarkerSize',8,'Color',colorc(1,:)*0+.5)
    for jj = 1:size(LiDARassoNavtmp,1)
        plot(Cyl(jj,1),Cyl(jj,3),'o','MarkerSize',8,'Color',colorc(jj,:))
        plot(LiDARassoNavtmp(jj,1),LiDARassoNavtmp(jj,2),'x','Color',colorc(jj,:))
    end
    set(gcf,'position',[1015 200 500 400])

    figure(11)
    colorc = colormap('jet'); % jet
    colorc = resample(colorc, nbCyl, size(colorc, 1));
    colorc(colorc>1)=1;colorc(colorc<0)=0;
    hold on
    %         plot(Cyl(:,4),'.','MarkerSize',8,'Color',colorc(1,:)*0+.5)
    for jj = 1:size(LiDARassoNavtmp,1)
        %           plot(jj,Cyl(jj,4),'o','MarkerSize',8,'Color',colorc(jj,:))
        plot(jj,LiDARassoNavtmp(jj,3),'x','Color',colorc(jj,:))
    end
    set(gcf,'position',[1515 200 500 400])
    set(gca,'xtick',-1:6)
    set(gca,'ylim',[0,260])
end
        %%%% IMU Figure %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        IMUpc=riv;
        IMUpc(:,1) = IMUpc(:,1);                
        
        [val1(jind),jindIMU] = min(abs(timeVec(jind)-IMUpc(:,1)));
        VICONpcSyncTV(jind,2:7) = VICONpc(indLiVImatch,2:7);% angX(pitch),angY(yaw), andZ(roll), posX, posY, posZ
        IMU_SyncTV(jind,1:6) = [[IMUpc(jindIMU,8) IMUpc(jindIMU,7) IMUpc(jindIMU,6)]...%*720/(2^31)*200,...%1/131.2*pi/180
                                [IMUpc(jindIMU,5) IMUpc(jindIMU,4) IMUpc(jindIMU,3)]];%*200/(2^31)*200];%*9.81/512];
        IMUangVec(jind,1:3) = [IMUpc(jindIMU,8) IMUpc(jindIMU,7) IMUpc(jindIMU,6)];%*720/(2^31)*200;%*1/131.2*pi/180; %1/131.2*pi/180;  %1/264.4*pi/180; %1e-4;%
        IMU_Iter{jind,1}(:,1:6) = [[IMUpc(jiterIMU:jindIMU-1,8) IMUpc(jiterIMU:jindIMU-1,7) IMUpc(jiterIMU:jindIMU-1,6)]...%*720/(2^31)*200,...%1/131.2*pi/180
                                  [IMUpc(jiterIMU:jindIMU-1,5) IMUpc(jiterIMU:jindIMU-1,4) IMUpc(jiterIMU:jindIMU-1,3)]];%*200/(2^31)*200];
        IMU_IterTime{jind,1}(:,1) = IMUpc(jiterIMU:jindIMU-1,1);
        jiterIMU=jindIMU;
        
        if flag_FEplot
            figure(7)
            plot((timeVec(1:jind)-timeVec(1))*1,IMUangVec(1:jind,1),'-k','Linewidth',2)
            hold on
            plot((timeVec(1:jind)-timeVec(1))*1,IMUangVec(1:jind,2),'-b','Linewidth',2)
            plot((timeVec(1:jind)-timeVec(1))*1,IMUangVec(1:jind,3),'-r','Linewidth',2)
            hold off
            %         set(gca,'XLim',[0 58],'YLim',[-0.8 0.8])
            xlabel('Time (s)')
            ylabel('Angular Velocity (deg/s)')
            legend('INS X-Axis','INS Y-Axis','INS Z-Axis','Location','SouthEast')
            set(gcf,'position',[515 200 500 400])
            F3(jind)=getframe(gcf);
            
        end
        %pause(.001)
end
        
       
figure(50)
%plot((timeVec(2:nbInc)-timeVec(1))*1,IMUangVec(2:nbInc,1),'-k','Linewidth',2)
plot((timeVec(2:nbInc)-timeVec(1))*1,IMUangVec(2:nbInc,3),'-r','Linewidth',1)
hold on
%plot((timeVec(2:nbInc)-timeVec(1))*1,IMUangVec(2:nbInc,2),'-b','Linewidth',2)
%plot((timeVec(2:nbInc)-timeVec(1))*1,rad2deg(pphi_rate(1:nbInc-1)),'--k','Linewidth',2)
%plot((timeVec(2:nbInc)-timeVec(1))*1,rad2deg(ttheta_rate(1:nbInc-1)),'--b','Linewidth',2)
plot((timeVec(2:nbInc)-timeVec(1))*1,rad2deg(psi_rate(1:nbInc-1)),'-k','Linewidth',1)

% IMU Figure    

toc
tic
% LiDARextr = [LiDARextr timeVec];
% video = VideoWriter('animation3D_1.mp4','MPEG-4');
% video1 = VideoWriter('LiDAR_PC.mp4','MPEG-4');
% video2 = VideoWriter('Circle_fitting.mp4','MPEG-4');
% video3 = VideoWriter('IMU_AngularVel.mp4','MPEG-4');
% video.FrameRate = 10;
% open(video);
% writeVideo(video,  F);
% close(video);

%%
% [val2,indIMUstat] = min(abs(vicontimestamps(1,1)-IMUpc(:,1)));
indIMUDyn=size(IMU_Iter{1,1},1);
IMUpcDy=IMUpc(indIMUDyn+1:end,:);
for ii=1:length(IMUpcDy)
% for ii=1:6000
    [val3,indIMUmatch] = min(abs(VICONpc (:,1)-IMUpcDy(ii,1)));
%     VICON_IMUSync(ii,1:6) = VICONpc(indIMUmatch,1:6);
    VICON_IMUSync(ii,1:6) = VICONpc(indIMUmatch,2:7);
%     VICON_IMUTimeSync(ii,1) = vicontimestamps(indIMUmatch,1);
    VICON_IMUTimeSync(ii,1) = VICONpc(indIMUmatch,1);
end
%% Don't delete 
%{

figure(8)
colorc = colormap('jet'); % jet
colorc = resample(colorc, nbCyl, size(colorc, 1));
colorc(colorc>1)=1;colorc(colorc<0)=0;
subplot(3,1,1)
plot(([timeVec(1) timeVec(end)]-timeVec(1)),[Cyl(:,1) Cyl(:,1)])
hold on
for jj = 1:nbCyl,
    plot(([timeVec(1) timeVec(end)]-timeVec(1)),[Cyl(jj,1) Cyl(jj,1)],'Color',colorc(jj,:))
    tmp = find(LiDARassoNav(:,2*jj-1)~=0);
    plot((timeVec(tmp)-timeVec(1)),LiDARassoNav(tmp,2*jj-1),'x','Color',colorc(jj,:),'Linewidth',2)
end
hold off
ylabel('X Assoc')
subplot(3,1,2)
plot(([timeVec(1) timeVec(end)]-timeVec(1)),[Cyl(:,3) Cyl(:,3)])
hold on
for jj = 1:nbCyl,
    plot(([timeVec(1) timeVec(end)]-timeVec(1)),[Cyl(jj,3) Cyl(jj,3)],'Color',colorc(jj,:))
    tmp = find(LiDARassoNav(:,2*jj-1)~=0);
    plot((timeVec(tmp)-timeVec(1)),LiDARassoNav(tmp,2*jj),'x','Color',colorc(jj,:),'Linewidth',2)
end
hold off
ylabel('Y Assoc')
subplot(3,1,3)
hold on
for jj = 1:nbCyl,
    tmp = find(LiDARassoError(:,jj)~=0);
    plot((timeVec(tmp)-timeVec(1)),LiDARassoError(tmp,jj)/1000,'x','Color',colorc(jj,:),'Linewidth',2)
end
hold off
ylabel('Error')
set(gcf,'position',[10+600+500 40+400 500 500])

%%        
figure(9)
if nbObj>0
    subplot(5,1,1:2)
    hold on
    cylCenterXlasVec1=cylCenterXlasVec;
    cylCenterYlasVec1=cylCenterYlasVec;
    cylCenterXlasVec1(cylCenterXlasVec==0)=NaN;
    cylCenterYlasVec1(cylCenterYlasVec==0)=NaN;
    plot(0:nbInc-1,cylCenterXlasVec1(1:nbInc,:),'bx')
    plot(0:nbInc-1,cylCenterYlasVec1(1:nbInc,:),'kv')
    title('DATA to ALI: Cyl. Radius and Coordinate Values in LiDAR Frame')
%     set(gca,'XLim',[0 1]*length(frm_len),'YLim',[-1 1]*rgeLim)
    set(gca,'XLim',[0 1]*length(LiDARpc),'YLim',[-1 1]*rgeLim)

    subplot(5,1,3:4)
    cylRadVec1=cylRadVec;
    cylRadVec1(cylRadVec==0)=NaN;
    hold on
    plot(0:nbInc-1,cylRadVec1(1:nbInc,:), 'kx', 'linewidth',1)
    ylabel('Radius')
    set(gca,'XLim',[0 1]*nbInc,'YLim',[0 1]*cylRadMax)
end

subplot(5,1,5)
hold on
plot(0:nbInc-1,nbObjVec(1:nbInc), 'b.', 'linewidth',3)
% set(gca,'XLim',[0 1]*length(frm_len),'YLim',[0 1]*maxNbObj)
set(gca,'XLim',[0 1]*length(LiDARpc),'YLim',[0 1]*maxNbObj)
set(gcf,'position',[10+600 40 500 400])

indtmp=find(VICONpcSyncTV(:,3)<-2*pi);
VICONpcSyncTV(indtmp,3)=VICONpcSyncTV(indtmp,3)+2*pi;

%% Saving data in the desired Navigation frame 
savingdata_flag =1;
if savingdata_flag
% from opti frame to nav frame
    rotOpti2Nav = [0 1 0;-1 0 0;0 0 1]*[1 0 0;0 0 -1;0 1 0];  % first -90 degree around x -second +90 around z
    VICONpc(:,5:7)= (rotOpti2Nav*VICONpc(:,5:7)')';
    VICONpc(:,2:4)= (rotOpti2Nav*VICONpc(:,2:4)')';    
    VICONpcSyncTV(:,5:7)= (rotOpti2Nav*VICONpcSyncTV(:,5:7)')';
    VICONpcSyncTV(:,2:4)= (rotOpti2Nav*VICONpcSyncTV(:,2:4)')';
    VICON_IMUSync(:,4:6)= (rotOpti2Nav*VICON_IMUSync(:,4:6)')';
    VICON_IMUSync(:,1:3)= (rotOpti2Nav*VICON_IMUSync(:,1:3)')';

    Cyl = (rotOpti2Nav*Cyl')';
    LiDARassoNav_new = LiDARassoNav;
    for i=1:6
        LiDARassoNavtemp = (rotOpti2Nav*[LiDARassoNav(:,i*2-1) zeros(size(LiDARassoNav,1),1) LiDARassoNav(:,i*2)]')';
        LiDARassoNav_new(:,i*2-1:i*2) = LiDARassoNavtemp(:,1:2);
    end
    LiDARassoNav(:,1:12)= LiDARassoNav_new(:,1:12);
% save('PointsExtraAssoc_09042019sstmp','Cyl','timeVec','LiDARasso','VICONpcSyncTV','IMU_SyncTV','IMU_Iter','IMU_IterTime','VICON_IMUSync','VICON_IMUTimeSync','LiDARRangeErrorTotal','LiDARbearingErrorTotal')
save('PointsExtraAssoc_06032022sstmp','LiDARpc','VICONpc','Cyl','timeVec','LiDARasso','LiDARassoNav','VICONpcSyncTV','IMU_SyncTV','IMU_Iter','IMU_IterTime','VICON_IMUSync','VICON_IMUTimeSync','LiDARRangeErrorTotal','LiDARbearingErrorTotal','-v7.3')
end
%%
% figure(10)
% for jj = 1:nbCyl,
%     subplot(6,1,jj)
%     tmp = find(LiDARassoError(:,jj)~=0 & LiDARassoError(:,jj) < 2000);
%     plot((timeVec(tmp)-timeVec(1)),LiDARassoError(tmp,jj)/1000,'-x','Color',colorc(jj,:),'Linewidth',2)
%     hold on
% end

for jj = 1:nbCyl
LiDARerrormean_x(1,jj)=mean(nonzeros(LiDARassoError_X(:,jj)));
LiDARerrormean_y(1,jj)=mean(nonzeros(LiDARassoError_Y(:,jj)));
LiDARerrorstd_x(1,jj)=std(nonzeros(LiDARassoError_X(:,jj)));
LiDARerrorstd_y(1,jj)=std(nonzeros(LiDARassoError_Y(:,jj)));
LiDARassoErrorstd(1,jj)=std(nonzeros(LiDARassoError(:,jj)*1e-3));
LiDARassoRangeErrorstd(1,jj)=std(nonzeros(LiDARRangeErrorTotal(:,jj)*1e-3));
LiDARassobearingErrorstd(1,jj)=std(nonzeros(LiDARbearingErrorTotal(:,jj)*180/pi));
end

% figure(12)
% for jj = 1:nbCyl,
%     subplot(3,2,jj)   
% %     xbins1 = 0:.04;
% %     hist(nonzeros(LiDARassoError(:,jj)*1e-3)) 
%     hhh=histogram(LiDARassoError(:,jj)*1e-3, 'normalization', 'pdf');
% %     hhh.FaceColor = [0 0.5 0.5];
%     axis([0 0.5 0 25]);
% %     Errortsd =['\sigma = ',num2str(LiDARassoErrorstd(1,jj)),' meter'];
% %     text(max(xlim)-.2,max(ylim)-10,Errortsd)
%     title(['Landmark Num ',num2str(jj)])
%     legend(['Hand Written Code \sigma = ',num2str(LiDARassoErrorstd(1,jj)),'meter'])
% %     legend(['Matlab routine Code \sigma = ',num2str(LiDARassoErrorstd(1,jj)),'meter'])
% end
% 


 
% figure(13)
% clear CenterBinRange pdfValuesRange
% for jj = 1:nbCyl,
%     subplot(3,2,jj)   
% %     hist(nonzeros(LiDARRangeErrorTotal(:,jj)*1e-3));
% %     nbinsRange = 30;
%     hh=histogram(nonzeros(LiDARRangeErrorTotal(:,jj)*1e-3), 'normalization', 'pdf');
%     CenterBinRange{jj,1}(1,:)=(hh.BinLimits(1)+(hh.BinWidth/2)):hh.BinWidth:(hh.BinLimits(2));
%     pdfValuesRange{jj,1}(1,:)=hh.Values;
%     hold on
%     plot(CenterBinRange{jj,:}(1,:), pdfValuesRange{jj,:}(1,:),'-r','LineWidth',2);
% %     axis([0 0.5 0 25]);
%     title(['Landmark Num ',num2str(jj)])
%     legend(['Range Error \sigma = ',num2str(LiDARassoRangeErrorstd(1,jj)),'meter'])
% end
% 
% figure(14)
% clear CenterBinBearing pdfValuesBearing
% for jj = 1:nbCyl,
%     subplot(3,2,jj)  
% %     hist(nonzeros(LiDARbearingErrorTotal(:,jj)*180/pi))
% %     nbinsBearing = 30;
%     hhh=histogram(LiDARbearingErrorTotal(:,jj)*180/pi, 'normalization', 'pdf');
%     CenterBinBearing{jj,1}(1,:)=(hhh.BinLimits(1)+(hhh.BinWidth/2)):hhh.BinWidth:(hhh.BinLimits(2));
%     pdfValuesBearing{jj,1}(1,:)=hhh.Values;
%     hold on
%     plot(CenterBinBearing{jj,:}(1,:), pdfValuesBearing{jj,:}(1,:),'-r','LineWidth',2);
% %     axis([0 0.5 0 25]);
%     title(['Landmark Num ',num2str(jj)])
%     legend(['Bearing Angle Code \sigma = ',num2str(LiDARassobearingErrorstd(1,jj)),'meter'])
% end

% 
% figure(15)
% % clear CenterBinRange pdfValuesRange
% for jj = 1:nbCyl,
%     subplot(3,2,jj)   
% %     hist(nonzeros(LiDARRangeErrorTotal(:,jj)*1e-3));
% %     nbinsRange = 30;
%     hh=histogram(LiDARRangeErrorTotal(:,jj)*1e-3, 'normalization', 'cdf');
% %     CenterBinRange{jj,1}(1,:)=(hh.BinLimits(1)+(hh.BinWidth/2)):hh.BinWidth:(hh.BinLimits(2));
% %     pdfValuesRange{jj,1}(1,:)=hh.Values;
% %     hold on
% %     plot(CenterBinRange{jj,:}(1,:), pdfValuesRange{jj,:}(1,:),'-r','LineWidth',2);
% %     axis([0 0.5 0 25]);
%     title(['Landmark Num ',num2str(jj)])
%     legend(['Range Error \sigma = ',num2str(LiDARassoRangeErrorstd(1,jj)),'meter'])
% end

%    figure(16)
%    clear CenterBinRangeTotal pdfValuesRangeTotal
%     hh_total=histogram(nonzeros(LiDARRangeErrorTotal(1:end)*1e-3), 'normalization', 'pdf');
%     CenterBinRangeTotal=(hh_total.BinLimits(1)+(hh_total.BinWidth/2)):hh_total.BinWidth:(hh_total.BinLimits(2));
%     pdfValuesRangeTotal=hh_total.Values;
%     hold on
%     plot(CenterBinRangeTotal, pdfValuesRangeTotal,'-r','LineWidth',2);
%     title('Range Error pdf (All Landmarks data) ')
% %     legend(['Range Error \sigma = ',num2str(LiDARassoRangeErrorstd(1,jj)),'meter'])
% 
%    figure(17)
%     clear CenterBinBearingTotal pdfValuesBearingTotal
%     hhh_total=histogram(nonzeros(LiDARbearingErrorTotal(1:end)*180/pi), 'normalization', 'pdf');
%     CenterBinBearingTotal{jj,1}(1,:)=(hhh_total.BinLimits(1)+(hhh_total.BinWidth/2)):hhh_total.BinWidth:(hhh_total.BinLimits(2));
%     pdfValuesBearingTotal{jj,1}(1,:)=hhh_total.Values;
%     hold on
%     plot(CenterBinBearingTotal{jj,:}(1,:), pdfValuesBearingTotal{jj,:}(1,:),'-r','LineWidth',2);
%    title('Bearing Angle Error pdf (All Landmarks data) ')
   
%    figure(18)
%     clear CenterBinRangeTotal pdfValuesRangeTotal
%     hh_total=histogram(nonzeros(LiDARRangeErrorTotal(1:end)*1e-3), 'normalization', 'cdf');
%     CenterBinRangeTotal{jj,1}(1,:)=(hh_total.BinLimits(1)+(hh_total.BinWidth/2)):hh_total.BinWidth:(hh_total.BinLimits(2));
%     pdfValuesRangeTotal{jj,1}(1,:)=hh_total.Values;
%     hold on
%     plot(CenterBinRangeTotal{jj,:}(1,:), pdfValuesRangeTotal{jj,:}(1,:),'-r','LineWidth',2);
%     title('Range Error cdf (All Landmarks data) ')
%     
%     figure(19)
%     clear CenterBinBearingTotal pdfValuesBearingTotal
%     hhh_total=histogram(nonzeros(LiDARbearingErrorTotal(1:end)*180/pi), 'normalization', 'cdf');
%     CenterBinBearingTotal{jj,1}(1,:)=(hhh_total.BinLimits(1)+(hhh_total.BinWidth/2)):hhh_total.BinWidth:(hhh_total.BinLimits(2));
%     pdfValuesBearingTotal{jj,1}(1,:)=hhh_total.Values;
%     hold on
%     plot(CenterBinBearingTotal{jj,:}(1,:), pdfValuesBearingTotal{jj,:}(1,:),'-r','LineWidth',2);
%    title('Bearing Angle Error cdf (All Landmarks data) ')
%    toc
   
if falg_polyfitremove;
    figure(20)
    clear P mupoly 
    for jj = 1:nbCyl,
        subplot(6,1,jj)
        tmp1 = find(LiDARRangeErrorTotal(:,jj)~=0);
        [P(:,jj),~,mupoly(:,jj)]=polyfit((timeVec(tmp1)-timeVec(1)),(LiDARRangeErrorTotal(tmp1,jj)/1000),8);
        f = polyval(P(:,jj)',(timeVec(tmp1)-timeVec(1)),[],mupoly(:,jj));
        plot((timeVec(tmp)-timeVec(1)),LiDARRangeErrorTotal(tmp,jj)/1000,'-x','Color',colorc(jj,:),'Linewidth',2)
        hold on
        plot((timeVec(tmp1)-timeVec(1)), f,'o','Color',colorc(jj,:),'Linewidth',1);
    end
%     clear P mupoly PP muppoly

    figure(21)
    clear CenterBinRange_poly pdfValuesRange_poly PP muppoly
    for jj = 1:nbCyl,
        subplot(3,2,jj) 
        tmp1 = find(LiDARRangeErrorTotal(:,jj)~=0);
        if jj<=4
        [PP{jj,1}(:,1),~,muppoly{jj,1}(:,1)]=polyfit((timeVec(tmp1)-timeVec(1)),(LiDARRangeErrorTotal(tmp1,jj)/1000),8);
        else
        [PP{jj,1}(:,1),~,muppoly{jj,1}(:,1)]=polyfit((timeVec(tmp1)-timeVec(1)),(LiDARRangeErrorTotal(tmp1,jj)/1000),11);
        end
        ff = polyval(PP{jj,1}(:,1)',(timeVec(tmp1)-timeVec(1)),[],muppoly{jj,1}(:,1));
        LiDARRangeErrorTotal_poly=((LiDARRangeErrorTotal(tmp1,jj)/1000)-ff);
        LiDARRangeErrorTotal_Poly{jj,1}(:,1)=LiDARRangeErrorTotal_poly;
        hh_poly=histogram(nonzeros( LiDARRangeErrorTotal_poly), 'normalization', 'pdf');
        CenterBinRange_poly{jj,1}(1,:)=(hh_poly.BinLimits(1)+(hh_poly.BinWidth/2)):hh_poly.BinWidth:(hh_poly.BinLimits(2));
        pdfValuesRange_poly{jj,1}(1,:)=hh_poly.Values;
        hold on
        plot(CenterBinRange_poly{jj,:}(1,:), pdfValuesRange_poly{jj,:}(1,:),'-r','LineWidth',2);
%     axis([0 0.5 0 25]);
        title(['Landmark Num ',num2str(jj)])
%         legend(['Range Error \sigma = ',num2str(LiDARassoRangeErrorstd(1,jj)),'meter'])
    end      
    
figure(22)
clear CenterBinBearing_poly pdfValuesBearing_poly PP muppoly
for jj = 1:nbCyl,
    subplot(3,2,jj)  
    tmp2 = find(LiDARbearingErrorTotal(:,jj)~=0);
     if jj<=3
        [PP{jj,1}(:,1),~,muppoly{jj,1}(:,1)]=polyfit((timeVec(tmp2)-timeVec(1)),(LiDARbearingErrorTotal(tmp2,jj)*180/pi),14);
     elseif jj==4
         [PP{jj,1}(:,1),~,muppoly{jj,1}(:,1)]=polyfit((timeVec(tmp2)-timeVec(1)),(LiDARbearingErrorTotal(tmp2,jj)*180/pi),14);
     else
        [PP{jj,1}(:,1),~,muppoly{jj,1}(:,1)]=polyfit((timeVec(tmp2)-timeVec(1)),(LiDARbearingErrorTotal(tmp2,jj)*180/pi),16);
     end
        ff = polyval(PP{jj,1}(:,1)',(timeVec(tmp2)-timeVec(1)),[],muppoly{jj,1}(:,1));
        LiDARbearingErrorTotal_poly=((LiDARbearingErrorTotal(tmp2,jj)*180/pi)-ff);
        LiDARbearingErrorTotal_Poly{jj,1}(:,1)=LiDARbearingErrorTotal_poly;
    hhh_poly=histogram(LiDARbearingErrorTotal_poly, 'normalization', 'pdf');
    CenterBinBearing_poly{jj,1}(1,:)=(hhh_poly.BinLimits(1)+(hhh_poly.BinWidth/2)):hhh_poly.BinWidth:(hhh_poly.BinLimits(2));
    pdfValuesBearing_poly{jj,1}(1,:)=hhh_poly.Values;
    hold on
    plot(CenterBinBearing_poly{jj,:}(1,:), pdfValuesBearing_poly{jj,:}(1,:),'-r','LineWidth',2);
% %     axis([0 0.5 0 25]);
    title(['Landmark Num ',num2str(jj)])
%     legend(['Bearing Angle Code \sigma = ',num2str(LiDARassobearingErrorstd(1,jj)),'meter'])
end

figure(23)
   clear CenterBinRangeTotal pdfValuesRangeTotal
   LiDARRangeErrorTotal_PPoly=cell2mat(LiDARRangeErrorTotal_Poly);
    hh_total_poly=histogram(nonzeros(LiDARRangeErrorTotal_PPoly(1:end)), 'normalization', 'pdf');
    CenterBinRangeTotal=(hh_total_poly.BinLimits(1)+(hh_total_poly.BinWidth/2)):hh_total_poly.BinWidth:(hh_total_poly.BinLimits(2));
    pdfValuesRangeTotal=hh_total_poly.Values;
    hold on
    plot(CenterBinRangeTotal, pdfValuesRangeTotal,'-r','LineWidth',2);
    title('Range Error pdf (All Landmarks data) ')
    
    figure(24)
    clear CenterBinBearingTotal pdfValuesBearingTotal
    LiDARbearingErrorTotal_PPoly=cell2mat(LiDARbearingErrorTotal_Poly);
    hhh_total_poly=histogram(nonzeros(LiDARbearingErrorTotal_PPoly(1:end)), 'normalization', 'pdf');
    CenterBinBearingTotal=(hhh_total_poly.BinLimits(1)+(hhh_total_poly.BinWidth/2)):hhh_total_poly.BinWidth:(hhh_total_poly.BinLimits(2));
    pdfValuesBearingTotal=hhh_total_poly.Values;
    hold on
    plot(CenterBinBearingTotal, pdfValuesBearingTotal,'-r','LineWidth',2);
    title('Bearing Error pdf (All Landmarks data) ')

end

 figure(18)
    clear CenterBinRangeTotal cdfValuesRangeTotal
    hh_total_poly=histogram(nonzeros(LiDARRangeErrorTotal_PPoly(1:end)), 'normalization', 'cdf');
    CenterBinRangeTotal{jj,1}(1,:)=(hh_total_poly.BinLimits(1)+(hh_total_poly.BinWidth/2)):hh_total_poly.BinWidth:(hh_total_poly.BinLimits(2));
    CdfValuesRangeTotal{jj,1}(1,:)=hh_total_poly.Values;
    hold on
    plot(CenterBinRangeTotal{jj,:}(1,:), CdfValuesRangeTotal{jj,:}(1,:),'-r','LineWidth',2);
    title('Range Error cdf (All Landmarks data) ')
    
    figure(19)
    clear CenterBinBearingTotal cdfValuesBearingTotal
    hhh_total_poly=histogram(nonzeros(LiDARbearingErrorTotal_PPoly(1:end)), 'normalization', 'cdf');
    CenterBinBearingTotal{jj,1}(1,:)=(hhh_total_poly.BinLimits(1)+(hhh_total_poly.BinWidth/2)):hhh_total_poly.BinWidth:(hhh_total_poly.BinLimits(2));
    CdfValuesBearingTotal{jj,1}(1,:)=hhh_total_poly.Values;
    hold on
    plot(CenterBinBearingTotal{jj,:}(1,:), CdfValuesBearingTotal{jj,:}(1,:),'-r','LineWidth',2);
   title('Bearing Angle Error cdf (All Landmarks data) ')
   toc
   
   figure(27)
   h27_1=qqplot(nonzeros(LiDARRangeErrorTotal_PPoly(1:end-60)));
   grid on
   brush on
   hold on
   h27_2=refline(.12,0);
   h27_2.Color = 'k';
   h27_2.LineWidth = 1;
   title('QQ Plot of Laser Range  Measurements Error versus Standard Normal')
   ylabel('Quantiles of Range Measurment Error')
   legend([h27_1(1),h27_2(1),h27_1(3)],'Range Measuremnet Error Sample','Single CDF Overbounding \sigma = 0.12 m','Normal Distribution');
%    hold on 
%    refline(.07,-.15)

   
   
   figure(28)
   LiDARbearingErrorTotal_PPoly(2082,1)=0;
   h28_1=qqplot(nonzeros(LiDARbearingErrorTotal_PPoly(1:end-60)));
   grid on
   hold on
   brush on
   h28_2=refline(2,0);
   h28_2.Color = 'k';
   h28_2.LineWidth = 1;
   title('QQ Plot of Laser Bearing Angle Measurements Error versus Standard Normal')
   ylabel('Quantiles of Bearing Angle Measurment Error')
   legend([h28_1(1),h28_2(1),h28_1(3)],'Bearing Angle Measuremnet Error Sample','Single CDF Overbounding \sigma = 2 deg','Normal Distribution');
   
%    figure(29)
%    h29_1=qqplot(nonzeros(cat(1,LiDAR_Intensity(:,2),LiDAR_Intensity(:,4),LiDAR_Intensity(:,5))-mean(cat(1,LiDAR_Intensity(:,2),LiDAR_Intensity(:,4),LiDAR_Intensity(:,5)))));
%    grid on
%    hold on
%    h29_2=refline(1.5,5);
%    h29_2.Color = 'k';
%    h29_2.LineWidth = 1;
%    hold on 
%    h29_3=refline(1.5,-5);
%    h29_3.Color = 'k';
%    h29_3.LineWidth = 1;
%    title('QQ Plot of Laser Intensity  Measurements Error for Black Color Surface ')
%    ylabel('Quantiles of Intensity Measurment Error')
%    legend([h29_1(1),h29_2(1),h29_1(3)],'Intensity Measuremnet Error','Pair CDF Overbounding m = 5 \sigma = 1.5','Normal Distribution');
%    
%    figure(30)
%    h30_1=qqplot(nonzeros(cat(1,LiDAR_Intensity(:,3),LiDAR_Intensity(:,6))-mean(cat(1,LiDAR_Intensity(:,3),LiDAR_Intensity(:,6)))));
%    grid on
%    hold on
%    h30_2=refline(6,12);
%    h30_2.Color = 'k';
%    h30_2.LineWidth = 1;
%    hold on 
%    h30_3=refline(6,-12);
%    h30_3.Color = 'k';
%    h30_3.LineWidth = 1;
%    title('QQ Plot of Laser Intensity  Measurements Error for White Color Surface ')
%    ylabel('Quantiles of Intensity Measurment Error')
%    legend([h30_1(1),h30_2(1),h30_1(3)],'Intensity Measuremnet Error','Pair CDF Overbounding m = 10 \sigma = 6','Normal Distribution');
%    
%    figure(31)
%    h31_1=qqplot(nonzeros(LiDAR_Intensity(:,1))-mean(LiDAR_Intensity(:,1)));
%    grid on
%    hold on
%    h31_2=refline(7,25);
%    h31_2.Color = 'k';
%    h31_2.LineWidth = 1;
%    hold on 
%    h31_3=refline(7,-25);
%    h31_3.Color = 'k';
%    h31_3.LineWidth = 1;
%    title('QQ Plot of Laser Intensity  Measurements Error for Retro Reflective Color Surface ')
%    ylabel('Quantiles of Intensity Measurment Error')
%    legend([h31_1(1),h31_2(1),h31_1(3)],'Intensity Measuremnet Error','Pair CDF Overbounding m = 25 \sigma = 7','Normal Distribution');
%  
  


%}  
   