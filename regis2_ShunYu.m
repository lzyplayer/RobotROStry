clc;clear;close all;
load("ShunYu8cloudsz.mat");
% profile on
% profile off
addpath('./flann/');
addpath('./estimateRigidTransform');
icpGridStep = 0.3;
eigDGridStep = 1.5;
overlap = 0.5;
icpToler= 0.5;

res= 10;

globalCameraPosition=[0,0,0];
relativeMotion{1}=eye(4);

tic;
% fig=figure('Position',[55 173 590 696]);
trclouds=cloudsTrim(clouds);
i=5;
[relativeMotion{i},MSE(i,1),tryTimes]=matchFix(trclouds{i-1},trclouds{i},overlap,eigDGridStep,res,i,icpToler);
disp(['fixtime: ' int2str(tryTimes) ])
disp(['MSE: '  num2str(MSE(i,1)) ])
regisTime=toc;
disp(['time(seconds): '  num2str(regisTime) ])
tosee=i;
pcshow(trclouds{tosee-1});hold on;
pcshow(pctransform(trclouds{tosee},affine3d(relativeMotion{tosee}')));%historyAccMotion{pairnum}'

% end
% for i=1:20
%     input('please input commond:\n','s');
%     clouds{i}=pcObj;
% end
% figure;axis on;
% while true
%     pcshow(pcObj);
%     pause(0.5);
% end

%% try Timer bad performence
% t = timer;
% t.ExecutionMode='fixedRate';
% t.Period=0.25;
% t.TimerFcn =@(myTimerObj, thisEvent)pcshow(pcObj);
% start(t);
% delete(t);% stop timer