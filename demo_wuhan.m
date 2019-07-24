clc;clear;close all;
load ("wuhan_surround_withoutground.mat");
load("wuhan1-85.mat");
addpath('./flann/');
addpath('./estimateRigidTransform');
%% argument
currLoop=0;
eigDGridStep = 1;
overlap = 0.7;
icpToler= 0.05;
mergeStep=0.05;
% ICPthreashold= 50;
maxPairDistance=2;
res= 10;
MSEHold=icpToler;
% s=1;
LoopDectNum=5;
loopMAmaxNum=90;
%% initial
LoopFlag=0;
cameraPosePair=[];
globalCameraPosition=[0,0,0];
relativeMotion{1}=eye(4);
MotionGlobal{1}=eye(4);
%%
for i=2:75
    relativeMotion{i}=(MotionGlobal{i-1})\MotionGlobal{i};
    globalCameraPosition(i,:)=MotionGlobal{i}(1:3,4)';
end
%%
hold on
% reScatter = reAxes.Children;
N=84;
for i=75:N
    fixS=clock;
    [relativeMotion{i},MSE(i,1),tryTimes]=matchFix(clouds{i-1},clouds{i},overlap,eigDGridStep,res,i,icpToler);
    MotionGlobal{i}=MotionGlobal{i-1}*relativeMotion{i};
    globalCameraPosition(i,:)=MotionGlobal{i}(1:3,4)';

    
    fixE=clock;
    fixtime=etime(fixE,fixS);
    disp(['curr fixtime: ' num2str(fixtime)])
    disp(['retry times: ' num2str(tryTimes)])
    disp('');
    
    %% 回环检测开始
    LoopPairNum=size(cameraPosePair,1);
    if(size(globalCameraPosition,1)>LoopDectNum)
        [cameraPosePair,LoopFlag]=estimateLoopFixed(globalCameraPosition,cameraPosePair,LoopDectNum,LoopFlag,maxPairDistance);
    end
    %% 回环结束检测_特征点匹配_匹配对扩展
    if((LoopPairNum==size(cameraPosePair,1) ||i==N) && (LoopFlag==1 ))
        currLoop=currLoop+1;
        routeDisplay(MotionGlobal,'b-*',true,[]);
        loopNumList(currLoop)=i;
        accMotion=fastDesEigMatch(clouds,cameraPosePair,overlap,eigDGridStep,res,MSEHold);
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
    end
end

    routeDisplay(MotionGlobal,'r-o',false,[]);
    obtainResult(clouds,MotionGlobal(1:60),false,mergeStep);
