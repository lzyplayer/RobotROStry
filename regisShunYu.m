clc;clear;close all;
load("ShunYu8clouds.mat");
% profile on
% profile off
addpath('./flann/');
addpath('./estimateRigidTransform');
icpGridStep = 0.3;
eigDGridStep = 2;
overlap = 0.5;
icpToler= 1;
mergeStep = 0.001;
res= 10;

globalCameraPosition=[0,0,0];
relativeMotion{1}=eye(4);
MotionGlobal{1}=eye(4);
% fig=figure('Position',[55 173 590 696]);
trclouds=cloudsTrim(clouds);
for i=2:length(clouds)
    disp(['process frame ' int2str(i) ])
    tic;
    [relativeMotion{i},MSE(i,1),tryTimes]=matchFix(trclouds{i-1},trclouds{i},overlap,eigDGridStep,res,i,icpToler);
    MotionGlobal{i}=MotionGlobal{i-1}*relativeMotion{i};
    globalCameraPosition(i,:)=MotionGlobal{i}(1:3,4)';
    disp(['fixtime: ' int2str(tryTimes) ])
    disp(['MSE: '  num2str(MSE(i,1)) ])
    regisTime=toc;
    disp(['time(seconds): '  num2str(regisTime) ])
    disp([' '])
    
end

routeDisplay(MotionGlobal,'r-o',false,[]);
obtainResult(trclouds,MotionGlobal,false,mergeStep);

% tosee=i;
% pcshow(trclouds{tosee-1});hold on;
% pcshow(pctransform(trclouds{tosee},affine3d(relativeMotion{tosee}')));%historyAccMotion{pairnum}'

%% try Timer bad performence
% t = timer;
% t.ExecutionMode='fixedRate';
% t.Period=0.25;
% t.TimerFcn =@(myTimerObj, thisEvent)pcshow(pcObj);
% start(t);
% delete(t);% stop timer