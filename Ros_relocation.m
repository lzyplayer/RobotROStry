clc;clear;close all;
load("ShunYuFullCloud.mat")
rosshutdown;
setenv('ROS_IP','192.168.1.101')
rosinit('192.168.1.103');
rosSuber=rossubscriber('/velodyne_points',@rosPCCallback);
global pcObj
while isempty(pcObj)
    pause(0.1);
    disp('Cannot get pointCloud !')
end
% profile on
% profile off
addpath('./flann/');
addpath('./estimateRigidTransform');
icpGridStep = 0.3;
eigDGridStep = 1.5;
overlap = 0.5;
icpToler= 0.5;
res= 10;
commond='';
% trclouds=cloudsTrim(clouds);
tic;
% fig=figure('Position',[55 173 590 696]);
downFullCloud = pcdownsample( fullcloud,'gridAverage',0.05);
%% select target
while  isempty(commond) %&& commond~='q'
    commond=input('please input commond(enter for next): \n','s');
    pause(6)
    if commond == 'q'
        break
    end
    %% downSample
    targetcloud = cloudsTrim(pcObj);
    downTargetCloud = pcdownsample( targetcloud,'gridAverage',0.05);

    %% relocate

    close all;
    % targetCloud
    [relativeMotion,MSE,tryTimes]=matchFix(downFullCloud,downTargetCloud,overlap,eigDGridStep,res,0,icpToler);
    disp(['fixtime: ' int2str(tryTimes) ])
    disp(['MSE: '  num2str(MSE) ])
    regisTime=toc;
    disp(['relocation time(seconds): '  num2str(regisTime) ])
    %% display
    displayTargetxzyPoints =  targetcloud.Location;
    displayTargetPointCloud = pointCloud(displayTargetxzyPoints,'color',zeros(size(displayTargetxzyPoints)));
    pcshow(downFullCloud);hold on;
    pcshow(pctransform(displayTargetPointCloud,affine3d(relativeMotion')));%historyAccMotion{pairnum}'
    routeDisplay(relativeMotion,'ro',false);
    theta = rotm2eul(relativeMotion(1:3,1:3),'XYZ');
    theta = theta(3);
    disp(['x: ' num2str(relativeMotion(1,4)) 'y: ' num2str(relativeMotion(2,4))]);
    disp(['theta: ' num2str(theta) 'pai']);
end