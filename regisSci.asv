clc;clear;close all;
addpath('./flann/');
addpath('./estimateRigidTransform');
load('scienceBuild_0-3_noGroundRoof.mat');
load('scienceBuild_0-3_trimmed.mat');
load('goodRelaBuilding.mat');
eigDGridStep = 0.7;
mergeGridStep = 0.05;
overlap = 0.8;
downSampleStep=0.01;
ICPthreashold= 50;
res= 1;
s=1;

MSEThresold=0.05;%初始，无意义
% trclouds = pcTrim(clouds,9,[-0.34 , 2],downSampleStep);
maxDis=3;
LoopDectNum=15;
loopMAmaxNum=12;
spacebetweenLoop=152;
currLoop=0;
LoopFlag=0;
lastLoopNum=1;
globalCameraPosition=[0,0,0];
relativeMotion{1}=eye(4);
MotionGlobal{1}=eye(4);
cameraPosePair=[];
historyCameraPosePair=[];
generalTime=tic;
historyAccMotion={};
alGetNum=length(relativeMotion);
fullCloud=trclouds{1};
figure('position',[-1439 76 1440 823]);
a=pcshow(fullCloud);
a.CameraPosition=[-20.573 -52.42 536.959];
a.CameraTarget=[4.579 18.606 0.83];
a.CameraUpVector=[0.331 0.933 0.139];
a.CameraViewAngle=5.1966;
% a.CameraPosition=[-374.242 -194.411 316.6];
% a.CameraTarget=[7.693 19.578 -1.906];
% a.CameraUpVector=[0.513 0.288 0.809];
% a.CameraViewAngle=3.4108;
Ascatter=a.Children;
axis off
%% show already had part

for i=2:alGetNum
    globalCameraPosition(i,:)=MotionGlobal{i}(1:3,4)';
    fullCloud = pcmerge(fullCloud,pcZTransMulti(trclouds{i},MotionGlobal{i}),mergeGridStep);
    Ascatter.XData=fullCloud.Location(:,1);
    Ascatter.YData=fullCloud.Location(:,2);
    Ascatter.ZData=fullCloud.Location(:,3);
    Ascatter.CData=fullCloud.Location(:,3);
    drawnow();
    
    pause(0.05);
end
%% continue register
nums=length(trclouds);
% while  isempty(commond) %&& commond~='q'
for i=i+1:nums
    
    
    
    
    disp(['processing frame: ' num2str(i)])
    fixS=clock;
    [relativeMotion{i},MSE(i,1),tryTimes]=matchFix(clouds{i-1},clouds{i},overlap,eigDGridStep,res,i,MSEThresold);
    MotionGlobal{i}=MotionGlobal{i-1}*relativeMotion{i};
    globalCameraPosition(i,:)=MotionGlobal{i}(1:3,4)';
    %     routeDisplay(MotionGlobal,'r-o',false,[]);
    %     obtainResult(trclouds,MotionGlobal,false,mergeGridStep);
    %     Ascatter.CameraPosition=[-3.224 -6.237 81.772];
    %     Ascatter.CameraTarget = [0.171 -0.003 0.634];
    %     Ascatter.CameraUpVector = [0.475 0.875 0.087];
    
    fixE=clock;
    Model = clouds{i-1}.Location';
    Data = clouds{i}.Location';
    [MSE(i,2),R,t] = TrICP(Model, Data, relativeMotion{i}(1:3,1:3), relativeMotion{i}(1:3,4), ICPthreashold, overlap);
    if(MSE(i,2) <= mean(MSE(i,1)))
        relativeMotion{i}=[[R,t];[0 0 0 1]];
    end
    fixtime=etime(fixE,fixS);
    disp(['curr fixtime: ' num2str(fixtime)])
    disp(['try times: ' num2str(tryTimes)])
    disp(['meanSquareERR: ' num2str(MSE(i,1))])
    disp('');
    
    
    
    
    %% 回环检测开始
    LoopPairNum=size(cameraPosePair,1);
    if(size(globalCameraPosition,1)>LoopDectNum && (i-lastLoopNum>spacebetweenLoop)) %防止两次修正太近
        [cameraPosePair,LoopFlag]=estimateLoop(globalCameraPosition,cameraPosePair,LoopDectNum,LoopFlag,lastLoopNum,maxDis);
    end
    %% 回环结束_特征点匹配_匹配对扩展
    if((i-lastLoopNum>spacebetweenLoop)&&(LoopPairNum==size(cameraPosePair,1) || LoopPairNum>=loopMAmaxNum || i==nums) && (LoopFlag==1 ))
        currLoop=currLoop+1;
        historyCameraPosePair=[historyCameraPosePair;[cameraPosePair,cameraPosePair(:,2)-lastLoopNum]];
        loopNumList(currLoop)=i;
        disp(['Loop ' num2str(currLoop)  ' detected completed, Motion Averaging starting...']);
        MotionGlobalBackup=MotionGlobal;
        accMotion=fastDesEigMatch(clouds,cameraPosePair,overlap,eigDGridStep,res,MSEThresold);
        if(isempty(accMotion))  %估测错误，特征点匹配并不好，
            LoopFlag=0;
            cameraPosePair=[];
            lastLoopNum=i;
            disp('wrong loop detected!')
            continue;
        end
        accMotion=[accMotion;historyAccMotion];
        historyAccMotion=accMotion;
        beforeMotion=(2:i);
        indiAccMotion=~cellfun(@(a) isempty(a) ,accMotion,'UniformOutput',true);
        trimmedMotion=accMotion(indiAccMotion(:,1),1:3);
        fixMotion=arrayfun(@(x) {relativeMotion{x},x-1,x} , beforeMotion ,'UniformOutput',false ); %补上前面fix结果
        fixMotion{length(fixMotion)+1}={eye(4),1,1};
        for f=1:length(fixMotion)
            trimmedMotion=[trimmedMotion;fixMotion{f}];
        end
        D=gen_Dij(trimmedMotion,length(relativeMotion),loopNumList);
        updatedGlobalMotion=MotionAverage(trimmedMotion,MotionGlobal,D,size(trimmedMotion,1),i,2,loopNumList);
        for k=1:length(updatedGlobalMotion)
            MotionGlobal{k}=updatedGlobalMotion{k};
        end
        
        LoopFlag=0;
        cameraPosePair=[];
        %         historyAccMotion={};
        lastLoopNum=i;
        %         maxPairDistance=100;
        %         LoopDectNum=290;
        disp(['Loop ' num2str(currLoop) ', Motion Averaging completed' ] );
        disp(' ');
        %%
        %         cla;
        disp('renew figure...');
        fullCloud=trclouds{1};
        for c=2:length(MotionGlobal)
            fullCloud=pcmerge(fullCloud,pcZTransMulti(trclouds{c},MotionGlobal{c}),mergeGridStep);
        end
        
        
        
        
    end
    
    %anime
    fullCloud = pcmerge(fullCloud,pcZTransMulti(trclouds{i},MotionGlobal{i}),mergeGridStep);
    Ascatter.XData=fullCloud.Location(:,1);
    Ascatter.YData=fullCloud.Location(:,2);
    Ascatter.ZData=fullCloud.Location(:,3);
    Ascatter.CData=fullCloud.Location(:,3);
    drawnow();
    %
    
end
% routeDisplay(MotionGlobal,'r-o',true,[154 1]);
% obtainResult(trclouds,MotionGlobal,false,mergeGridStep);
% pcTrim(trclouds,20,[-0.34 , 2],0.0001)

