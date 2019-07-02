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
pcselect  = select(bag,'Time',[bag.StartTime+15 bag.StartTime+17]);
pcmsgs = readMessages(pcselect);
pcObj  =pointCloud(readXYZ(pcmsgs{curr}));
%%
addpath('./flann/');
addpath('./estimateRigidTransform');
eigDGridStep = 0.7;
overlap = 0.9;
icpToler= 0.2;
res= 1;
downsampleSize= 0.05;
icpThersold=100;
commond='';

% fig=figure('Position',[55 173 590 696]);
%% generate map descriptor
downFullCloud = pcdownsample( fullcloud,'gridAverage',downsampleSize);
[mapDesp,mapSeed,mapNorm,mapDesNum] = extractEig(downFullCloud,eigDGridStep);
%%%%%%%%%%%要查询mapDesNum来了解匹配过程
%% select target
mapInfo={mapDesp,mapSeed,mapNorm};
commond = [];
while  isempty(commond) %&& commond~='q'
% % %     commond=input('please input commond(enter for next): \n','s');
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
    %% trimmed raw postion input 
%     piror_location = [0 ,5 ]; %x,y
%     select_map_Idx = rangesearch(mapSeed(1:2,:)',piror_location,20);
%     mapInfo = {mapDesp(:,select_map_Idx{1}),mapSeed(:,select_map_Idx{1}),mapNorm(:,select_map_Idx{1})} ;
    %% relocate
    close all;
    [relativeMotion,MSE,T]=simpleRegWithEig(mapInfo,currInfo,downFullCloud,downTargetCloud,overlap,eigDGridStep,res,icpToler,icpThersold);
    disp(['MSE: '  num2str(MSE) ])
    regisTime=toc;
    disp(['relocation time(seconds): '  num2str(regisTime) ])
    %% display
    displayTargetxzyPoints =  targetcloud.Location;
    displayTargetPointCloud = pointCloud(displayTargetxzyPoints,'color',zeros(size(displayTargetxzyPoints)));
    currAxes = pcshow(downFullCloud);hold on;
    pcshow(pctransform(displayTargetPointCloud,affine3d(relativeMotion')));
    routeDisplay({relativeMotion},'ro',false);
    
    theta = rotm2eul(relativeMotion(1:3,1:3),'XYZ');
    theta = theta(3);
    disp(['x: ' num2str(relativeMotion(1,4)) 'y: ' num2str(relativeMotion(2,4))]);
    RotRad = theta/pi;
    disp(['theta: ' num2str(RotRad*180) ' degree']);
    currAxes.Color=[1,1,1];
    commond='q';
end
%%
figure;
currAxes = pcshow(downFullCloud);hold on;
pcshow(pctransform(displayTargetPointCloud,affine3d(T')));
routeDisplay({T},'ro',false);
currAxes.Color=[1,1,1];