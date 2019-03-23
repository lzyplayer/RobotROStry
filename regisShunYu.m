clc;clear;close all;
load("ShunYu66.mat");
% profile on
% profile off
addpath('./flann/');
addpath('./estimateRigidTransform');
%% argument
icpGridStep = 0.3;
eigDGridStep = 2;
overlap = 0.7;
icpToler= 0.7;
mergeStep = 0.05;
res= 10;
maxPairDistance=3.5;
LoopDectNum=5;
loopMAmaxNum=8;
%% initial
currLoop = 0;
LoopFlag=0;
cameraPosePair=[];
globalCameraPosition=[0,0,0];
relativeMotion{1}=eye(4);
MotionGlobal{1}=eye(4);
trclouds=cloudsTrim(clouds);
%% axes
fullcloud = trclouds{1};
reAxes=pcshow(fullcloud);
title('Slam On 舜')
reScatter = reAxes.Children;

%% loop
N=length(trclouds);
for i=2:N %2:
    disp(['process frame ' int2str(i) ])
    tic;
    [relativeMotion{i},MSE(i,1),tryTimes]=matchFix(trclouds{i-1},trclouds{i},overlap,eigDGridStep,res,i,icpToler);
    MotionGlobal{i}=MotionGlobal{i-1}*relativeMotion{i};
    globalCameraPosition(i,:)=MotionGlobal{i}(1:3,4)';
    disp(['fixtime: ' int2str(tryTimes) ])
    disp(['MSE: '  num2str(MSE(i,1)) ])
    regisTime=toc;
    disp(['time(seconds): '  num2str(regisTime) ])
    disp(' ')
    
    %% anime
    fullcloud = pcmerge(fullcloud,pcZTransMulti(trclouds{i},MotionGlobal{i}),mergeStep);
    reScatter.XData=fullcloud.Location(:,1);
    reScatter.YData=fullcloud.Location(:,2);
    reScatter.ZData=fullcloud.Location(:,3);
    reScatter.CData=fullcloud.Location(:,3);
    drawnow()
    %% 回环检测开始
    LoopPairNum=size(cameraPosePair,1);
    if(size(globalCameraPosition,1)>LoopDectNum)
        [cameraPosePair,LoopFlag]=estimateLoopFixed(globalCameraPosition,cameraPosePair,LoopDectNum,LoopFlag,maxPairDistance);
    end
    %% 回环结束检测_特征点匹配_匹配对扩展
    if((LoopPairNum==size(cameraPosePair,1) || LoopPairNum>=loopMAmaxNum ||i==N) && (LoopFlag==1 ))
        currLoop=currLoop+1;
        disp(['Loop ' num2str(currLoop)  ' detected completed, Motion Averaging starting...']);
        %         routeDisplay(MotionGlobal,'b-*',true,[]);
        loopNumList(currLoop)=i;
        accMotion=fastDesEigMatch(trclouds,cameraPosePair,overlap,eigDGridStep,res,icpToler);
        beforeMotion=(2:length(relativeMotion));
        fixMotion=arrayfun(@(x) {relativeMotion{x},x-1,x,MSE(x)} , beforeMotion ,'UniformOutput',false ); %补上前面fix结果
        for f=1:length(fixMotion)
            accMotion=[accMotion;fixMotion{f}];
        end
        D=gen_Dij(accMotion,i,loopNumList);
        updatedGlobalMotion=MotionAverage(accMotion,MotionGlobal,D,size(accMotion,1),i,2,loopNumList);
        for k=1:length(updatedGlobalMotion)
            MotionGlobal{k}=updatedGlobalMotion{k};
        end
        LoopFlag=0;
        disp(['Loop ' num2str(currLoop) ', Motion Averaging completed' ] );
        disp(' ');
        %% anime
        cla;
        %       routeDisplay(GrtM,'g-d',false,[]);
        %         routeHandle=routeAnimePlugin(MotionGlobal,'r-o',routeAxes);
        disp('renew figure...');
        fullcloud=trclouds{1};
        for c=2:length(MotionGlobal)
            fullcloud=pcmerge(fullcloud,pcZTransMulti(trclouds{c},MotionGlobal{c}),mergeStep);
        end
        
        %         disp(' ICP forward registering...'  );
    end
end
pcshow(fullcloud);
% routeDisplay(MotionGlobal,'r-o',false,[40,30]);
% obtainResult(trclouds,MotionGlobal,false,mergeStep);

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