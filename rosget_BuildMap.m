clc;clear;close all;
rosshutdown;
setenv('ROS_IP','192.168.1.138')
rosinit('192.168.1.49');
rosSuber=rossubscriber('/velodyne_points',@rosPCCallback);
global pcObj
% if ~isempty(pcObj)
%     pcshow(pcObj);
% end
while isempty(pcObj)
    pause(0.1);
    disp('Cannot get pointCloud !')
end
addpath('./flann/');
addpath('./estimateRigidTransform');
%% argument
icpGridStep = 0.3;
eigDGridStep = 2;
overlap = 0.7;
icpToler= 0.5;
mergeStep = 0.015;
% ICPthreashold= 50;
maxPairDistance=0.5;
res= 10;
MSEHold=1;
% s=1;
LoopDectNum=5;
loopMAmaxNum=8;
%% initial
LoopFlag=0;
cameraPosePair=[];
globalCameraPosition=[0,0,0];
relativeMotion{1}=eye(4);
MotionGlobal{1}=eye(4);
%%
% generalTime=tic;
commond='';
clouds{1}=pcObj;
fig=figure('Position',[55 173 590 696]);
routeDisplay(MotionGlobal,'r-o',false,[]);
fullCloud=clouds{1};

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
    fixS=clock;
    [relativeMotion{i},MSE(i,1),tryTimes]=matchFix(clouds{i-1},clouds{i},overlap,eigDGridStep,res,i,icpToler);
    MotionGlobal{i}=MotionGlobal{i-1}*relativeMotion{i};
    globalCameraPosition(i,:)=MotionGlobal{i}(1:3,4)';
    
    prevCP=reAxes.CameraPosition;
    prevCT=reAxes.CameraTarget;
    prevCUV=reAxes.CameraUpVector;
    
    routeDisplay(MotionGlobal,'r-o',false,[]);
    obtainResult(clouds,MotionGlobal,false,mergeStep);
    
    %     reAxes.CameraPosition=prevCP;
    %     reAxes.CameraTarget=prevCT;
    %     reAxes.CameraUpVector=prevCUV;
    
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
    if((LoopPairNum==size(cameraPosePair,1) || LoopPairNum>=loopMAmaxNum) && (LoopFlag==1 ))
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