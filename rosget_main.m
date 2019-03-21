clc;clear;close all;
rosshutdown;
setenv('ROS_IP','192.168.1.101')
rosinit('192.168.1.103');
rosSuber=rossubscriber('/velodyne_points',@rosPCCallback);
global pcObj
while isempty(pcObj)
    pause(0.1);
    disp('Cannot get pointCloud !')
end
addpath('./flann/');
addpath('./estimateRigidTransform');

eigDGridStep = 2;
overlap = 0.5;
icpToler= 0.9;
mergeStep = 0.015;%-
% ICPthreashold= 50;
% maxPairDistance=0.1;
res= 10;
% s=1;
% LoopDectNum=15;
globalCameraPosition=[0,0,0];
relativeMotion{1}=eye(4);
MotionGlobal{1}=eye(4);
% generalTime=tic;
commond='';
trclouds{1}=pcObj;
% fig=figure('Position',[55 173 590 696]);
routeDisplay(MotionGlobal,'r-o',false,[]);
fullCloud=trclouds{1};

reAxes=pcshow(fullCloud);
hold on
% reScatter = reAxes.Children;
i=1;
while  isempty(commond) %&& commond~='q'
    commond=input('please input commond(enter for next): \n','s');
    if commond == 'q'
        break
    end
    i=i+1;
    clouds{i}=pcObj;
    trclouds{i}=cloudsTrim(clouds{i});
    %% need trim here
    fixS=clock;
    [relativeMotion{i},MSE(i,1),tryTimes]=matchFix(trclouds{i-1},trclouds{i},overlap,eigDGridStep,res,i,icpToler);
    MotionGlobal{i}=MotionGlobal{i-1}*relativeMotion{i};
    globalCameraPosition(i,:)=MotionGlobal{i}(1:3,4)';
%     
%     prevCP=reAxes.CameraPosition;
%     prevCT=reAxes.CameraTarget;
%     prevCUV=reAxes.CameraUpVector;
    
    routeDisplay(MotionGlobal,'r-o',false,[]);
    obtainResult(trclouds,MotionGlobal,false,mergeStep);
    
    %     reAxes.CameraPosition=prevCP;
    %     reAxes.CameraTarget=prevCT;
    %     reAxes.CameraUpVector=prevCUV;
    
    fixE=clock;
    fixtime=etime(fixE,fixS);
    disp(['curr fixtime: ' num2str(fixtime)])
    disp(['retry times: ' num2str(tryTimes)])
    disp('');
end


%
rosshutdown
clear('rosSuber');



%% try Timer but bad performence
% t = timer;
% t.ExecutionMode='fixedRate';
% t.Period=0.25;
% t.TimerFcn =@(myTimerObj, thisEvent)pcshow(pcObj);
% start(t);
% delete(t);% stop timer