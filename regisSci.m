clc;clear;close all;
addpath('./flann/');
addpath('./estimateRigidTransform');
load('scienceBuild_raw1-2.mat')
downSampleStep=0.005;
icpGridStep = 0.3;
eigDGridStep = 0.4;
mergeGridStep = 0.1;
overlap = 0.7;
icpToler= 1e-4;
ICPthreashold= 50;
maxPairDistance=0.1;
res= 1;
s=1;
trclouds = pcTrim(clouds,9,[-0.34 , 2],downSampleStep);
LoopDectNum=15;
globalCameraPosition=[0,0,0];
relativeMotion{1}=eye(4);
MotionGlobal{1}=eye(4);
generalTime=tic;


figure();
a=axes;

% fig=figure('Position',[55 173 590 696]);
% routeDisplay(MotionGlobal,'r-o',false,[]);
fullCloud=trclouds{1};
% reAxes=pcshow(fullCloud);
% hold on
% reScatter = reAxes.Children;
i=1;
nums=length(trclouds);
% while  isempty(commond) %&& commond~='q'
for i=2:nums
     
    if i==21
        eigDGridStep = 0.6;
    end   
    disp(['processing frame: ' num2str(i)])
    fixS=clock;
    [relativeMotion{i},MSE(i,1),tryTimes]=matchFix(trclouds{i-1},trclouds{i},overlap,eigDGridStep,res,i);
    MotionGlobal{i}=MotionGlobal{i-1}*relativeMotion{i};
    globalCameraPosition(i,:)=MotionGlobal{i}(1:3,4)';
    routeDisplay(MotionGlobal,'r-o',false,[]);
    obtainResult(trclouds,MotionGlobal,false,mergeGridStep);
    a.CameraPosition=[-3.224 -6.237 81.772];
    a.CameraTarget = [0.171 -0.003 0.634];
    a.CameraUpVector = [0.475 0.875 0.087];
    fixE=clock;

    Model = trclouds{i-1}.Location';
    Data = trclouds{i}.Location';
    [MSE(i,2),R,t] = TrICP(Model, Data, relativeMotion{i}(1:3,1:3), relativeMotion{i}(1:3,4), ICPthreashold, overlap);
    if(MSE(i,2) <= mean(MSE(i,1)))
        relativeMotion{i}=[[R,t];[0 0 0 1]];
    end
    fixtime=etime(fixE,fixS);
    disp(['curr fixtime: ' num2str(fixtime)])
    disp(['try times: ' num2str(tryTimes)])
    disp(['meanSquareERR: ' num2str(MSE(i,1))])
    disp('');
end
routeDisplay(MotionGlobal(1:34),'r-o',false,[21]);
obtainResult(trclouds,MotionGlobal(1:34),false,mergeGridStep);
% pcTrim(trclouds,20,[-0.34 , 2],0.0001)

