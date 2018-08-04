clc;clear;close all;

rosinit('192.168.1.187');
rosSuber=rossubscriber('/velodyne_points',@rosPCCallback);
global pcObj
% if ~isempty(pcObj)
%     pcshow(pcObj);
% end
while isempty(pcObj)
    pause(0.1);
end
addpath('./flann/');
addpath('./estimateRigidTransform');
icpGridStep = 0.3;
eigDGridStep = 0.4;
overlap = 0.5;
icpToler= 1e-4;
ICPthreashold= 50;
maxPairDistance=0.1;
res= 10;
s=1;
LoopDectNum=15;
globalCameraPosition=[0,0,0];
relativeMotion{1}=eye(4);
MotionGlobal{1}=eye(4);
generalTime=tic;
commond='';
clouds{1}=pcObj;
routeDisplay(MotionGlobal,'r-o',false,[]);
pcshow(clouds{1})
i=1;
while  isempty(commond) %&& commond~='q'
    commond=input('please input commond(enter for next): \n','s');
    close all;
    i=i+1;
    clouds{i}=pcObj;
    fixS=clock;
    [relativeMotion{i},MSE(i,1),tryTimes]=matchFix(clouds{i-1},clouds{i},overlap,eigDGridStep,res,i);
    MotionGlobal{i}=MotionGlobal{i-1}*relativeMotion{i};
    globalCameraPosition(i,:)=MotionGlobal{i}(1:3,4)';
    routeDisplay(MotionGlobal,'r-o',false,[]);
    obtainResult(clouds,MotionGlobal,false);
    fixE=clock;
    fixtime=etime(fixE,fixS);
    disp(['curr fixtime: ' num2str(fixtime)])
    disp(['retry times: ' num2str(tryTimes)])
    disp('');
end


%
rosshutdown
clear('rosSuber');


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