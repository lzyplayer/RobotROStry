clc;clear;close all;
load("ShunYuFullCloud.mat")
%% realTime Ros
% % rosshutdown;
% % setenv('ROS_IP','192.168.1.101')
% % rosinit('192.168.1.103');
% % rosSuber=rossubscriber('/velodyne_points',@rosPCCallback);
% % global pcObj
% % while isempty(pcObj)
% %     pause(0.1);
% %     disp('Cannot get pointCloud !')
% % end
% profile on
% profile off
%% get from rosbag
curr = 1;
bag = rosbag('./rosbag/shunyu.bag');
pcselect  = select(bag,'Time',[bag.StartTime+60 bag.StartTime+62]);
pcmsgs = readMessages(pcselect);
pcObj  =pointCloud(readXYZ(pcmsgs{curr}));
%%
addpath('./flann/');
addpath('./estimateRigidTransform');
eigDGridStep = 0.5;
overlap = 0.8;
icpToler= 0.2;
res= 1;
downsampleSize= 0.05;
icpThersold=50;
commond='';

% fig=figure('Position',[55 173 590 696]);
%% generate map descriptor
downFullCloud = pcdownsample( fullcloud,'gridAverage',downsampleSize);
[mapDesp,mapSeed,mapNorm] = extractEig(downFullCloud,eigDGridStep);
mapInfo={mapDesp,mapSeed,mapNorm};
%% select target
commond = [];
while  isempty(commond) %&& commond~='q'
    commond=input('please input commond(enter for next): \n','s');
%     pause(6)
    if commond == 'q'
        break
    end
    tic;
    %% downSample and generate curr descriptor
    targetcloud = cloudsTrim(pcObj);
    downTargetCloud = pcdownsample( targetcloud,'gridAverage',downsampleSize);
    [currDesp,currSeed,currNorm] = extractEig(downTargetCloud,eigDGridStep);
    currInfo={currDesp,currSeed,currNorm};
    %% relocate
    close all;
    [relativeMotion,MSE]=simpleRegWithEig(mapInfo,currInfo,downFullCloud,downTargetCloud,overlap,eigDGridStep,res,icpToler);
    disp(['MSE: '  num2str(MSE) ])
    regisTime=toc;
    disp(['relocation time(seconds): '  num2str(regisTime) ])
    %% display
    displayTargetxzyPoints =  targetcloud.Location;
    displayTargetPointCloud = pointCloud(displayTargetxzyPoints,'color',zeros(size(displayTargetxzyPoints)));
    currAxes = pcshow(downFullCloud);hold on;
    pcshow(pctransform(displayTargetPointCloud,affine3d(relativeMotion')));%historyAccMotion{pairnum}'
    routeDisplay({relativeMotion},'ro',false);
    
    theta = rotm2eul(relativeMotion(1:3,1:3),'XYZ');
    theta = theta(3);
    disp(['x: ' num2str(relativeMotion(1,4)) 'y: ' num2str(relativeMotion(2,4))]);
    RotRad = theta/pi;
    disp(['theta: ' num2str(RotRad*180) ' degree']);
    currAxes.Color=[1,1,1];
end