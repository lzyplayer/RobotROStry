clc;clear;close all;
load("ShunYu66.mat");
load("ShunYuFullCloud.mat")
% profile on
% profile off
addpath('./flann/');
addpath('./estimateRigidTransform');
icpGridStep = 0.3;
eigDGridStep = 1.5;
overlap = 0.5;
icpToler= 0.5;
res= 10;
trclouds=cloudsTrim(clouds);
tic;
% fig=figure('Position',[55 173 590 696]);
%% select target
i=15;
%% downSample
targetCloud = pcdownsample( trclouds{i},'gridAverage',0.05);
downFullCloud = pcdownsample( fullcloud,'gridAverage',0.05);
%% relocate


% targetCloud
[relativeMotion,MSE(i,1),tryTimes]=matchFix(downFullCloud,targetCloud,overlap,eigDGridStep,res,i,icpToler);
disp(['fixtime: ' int2str(tryTimes) ])
disp(['MSE: '  num2str(MSE(i,1)) ])
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