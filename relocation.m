clear;close all;%clc;
load("ShunYu8clouds.mat");
load("ShunYuFullCloud.mat");
% profile on
% profile off
addpath('./flann/');
addpath('./estimateRigidTransform');
icpGridStep = 0.3;
eigDGridStep = 2;
overlap = 0.5;
icpToler= 0.1;
res= 1;
trclouds=cloudsTrim(clouds);
tic;
% fig=figure('Position',[55 173 590 696]);
%% select target
i=6;
%% downSample
targetCloud = pcdownsample( trclouds{i},'gridAverage',0.1);
downFullCloud = pcdownsample( fullcloud,'gridAverage',0.1);
%% relocate


% targetCloud
[relativeMotion,MSE(i,1),tryTimes]=matchFix(downFullCloud,targetCloud,overlap,eigDGridStep,res,i,icpToler);
disp(['rawMSE: '  num2str(MSE(i,1)) ])
R0= relativeMotion(1:3,1:3);
t0= relativeMotion(1:3,4);
Model= fullcloud.Location(1:res:end,:)';
Data= trclouds{i}.Location(1:res:end,:)';
[MSE(i,1),R,t] = TrICP(Model, Data, R0, t0, 50, overlap);
relativeMotion(1:3,1:3) = R;
relativeMotion(1:3,4) = t;
disp(['fixtime: ' int2str(tryTimes) ])
disp(['icpMSE: '  num2str(MSE(i,1)) ])
regisTime=toc;
disp(['relocation time(seconds): '  num2str(regisTime) ])
%% display
displayTargetxzyPoints =  trclouds{i}.Location;
displayTargetPointCloud = pointCloud(displayTargetxzyPoints,'color',zeros(size(displayTargetxzyPoints)));
pcshow(downFullCloud);hold on;
pcshow(pctransform(displayTargetPointCloud,affine3d(relativeMotion')));%historyAccMotion{pairnum}'
routeDisplay(relativeMotion,'ro',false);
theta = rotm2eul(relativeMotion(1:3,1:3),'XYZ');
theta = theta(3);
disp(['theta: ' num2str(theta) 'pai']);