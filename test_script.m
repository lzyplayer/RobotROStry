clc;clear;close all;

%% 化为点云格式
for i=1:length(clouds)
    clouds{i}=pointCloud(clouds{i});
end
%% 去除噪声
load('scienceBuild_raw0-3.mat')
scannum=length(clouds);
for i=1:scannum
    indata=clouds{i}.Location;
    pointsFar=sqrt(indata(:,1).^2+indata(:,2).^2)>4.5;
    pointsGround=indata(:,3)<-0.4;
    pointRoof=indata(:,3)>2.1;
    pointSelect=~(pointsGround|pointRoof); %~(pointsFar | pointsGround);
    clouds{i}=pointCloud(indata(pointSelect,1:3));
end
%% 化绝对为相对
relativeMotionRe=cell(1,length(MotionGlobal));
relativeMotionRe{1}=eye(4);
for i=2:length(MotionGlobal)
    relativeMotionRe{i} = MotionGlobal{i-1} \ MotionGlobal{i} ;
end

%% 化相对为绝对
close all;
figure('position',[-1439 76 1440 823])
MotionGlobal=cell(1,length(relativeMotion));
MotionGlobal{1}=eye(4);
for i=2:length(relativeMotion)
    MotionGlobal{i} = MotionGlobal{i-1} * relativeMotion{i} ;
end
% optional
mergeGridStep = 0.1;
% routeDisplay(MotionGlobalRe,'r-o',false,[50,55,60]);
obtainResult(trclouds,MotionGlobal,false,mergeGridStep);

%% 替换破损帧
problemFrame=76;
relativeMotion{problemFrame} = eye(4);
relativeMotion{problemFrame+1} = Motion;
clouds{problemFrame}=clouds{problemFrame-1};

%% 真值处理
load hannover_GrtM_z_ConvertNeed.mat
% load hannover2_GrtM_ori.mat
load hannover2_result.mat
load hannover2_MZ.mat
% 重组为矩阵
% for i=1:size(GrtM_ori,1)
%     GrtM_ori_m{i}=reshape( GrtM_ori(i,:),4,4);
% end
% 
nums=length(GrtM);
for i=1:nums
    GrtM{i}=(GrtM{1}) \ GrtM{i};
end
mirrorM=[1 0 0 0;
         0 1 0 0;
         0 -1 1 0;
         0 0 0 1];

% GrtM_RelativeTo1=cell(nums,1);
GrtMatrixRightHand=cellfun(@(x) mirrorM*x,GrtM,'UniformOutput',0);

% %%     new
%     tfrom=MotionGlobal{47}*inv((GrtM_RelativeTo1{47}));
%     rot=tfrom(1:3,1:3);
for i=1:903
%     newMotion{i}=[rot*GrtM_RelativeTo1{i}(1:3,1:3),MotionGlobal{i}(1:3,4);0 0 0 1];
    GrtMatrixRightHand{i}(1:3,1:3)*GrtMatrixRightHand{i}(1:3,1:3)'
end
% obtain_model_hannover1(clouds,newMotion);
%     
%%
for i=1:length(GrtM_RelativeTo1)
    %% new
    resu{i}=rotm2quat(GrtM_RelativeTo1{i}(1:3,1:3));
    resu1{i}=rotm2quat(MotionGlobal{i}(1:3,1:3));
%% old
    EulerIn{i}=rotm2eul(GrtM_RelativeTo1{i}(1:3,1:3),'XYZ');
    EulerIn2{i}=rotm2eul(GrtM_RelativeTo1{i}(1:3,1:3),'ZYX');
    EulerNeed{i}=rotm2eul(MotionGlobal{i}(1:3,1:3),'XYZ');
    conEuler=[EulerIn{i}(3),EulerIn{i}(1),EulerIn{i}(2)];
    conRot=eul2rotm(conEuler,'XYZ');
    conTran=[GrtM_RelativeTo1{i}(3,4);GrtM_RelativeTo1{i}(1,4);GrtM_RelativeTo1{i}(2,4)];
    conGrtM{i}=Rt2M(conRot,conTran);
     EulerIn{i}=rotm2eul(conGrtM{i}(1:3,1:3),'ZYZ');

end
% obtain_model_hannover1(clouds,newMotion);


%% 相对位置变化限制
for i=2:length(relativeMotion)
    distance(i)=norm(  relativeMotion{i}(1:3,4));
end
%% 展示historyAccMotion中一对
figure;
toseeACC=8;
low=historyAccMotion{toseeACC,2};
high=historyAccMotion{toseeACC,3};
pcshow(clouds{low});hold on;
pcshow(pctransform(clouds{high},affine3d(historyAccMotion{toseeACC}')));%historyAccMotion{pairnum}'


%% 分别展示2帧
figure;
a=17;
pcshow(trclouds{a});
% pcshow(pctransform( clouds{a},affine3d(MotionGlobal{a}')));
hold on;
figure;
b=16;
pcshow(trclouds{b})
% pcshow(pctransform( clouds{b},affine3d(MotionGlobal{b}')));

% pc自带ICp算法
gridstep=0.01;
readnum=31;
overlap = 0.4;
res= 1;
s=30;
filepath='./data/local_frame/';
filePrefix='Hokuyo_';
load trimOutside;
% 自带配准效果不好
readyCloud1=pcdownsample(pcdenoise(clouds{4}),'gridAverage',gridstep);
readyCloud2=pcdownsample(pcdenoise(clouds{3}),'gridAverage',gridstep);
[motion,result]=pcregrigid(readyCloud2,readyCloud1,'Tolerance',[0.01/s,0.009]);
pcshow(result);
hold on;

% 相邻展现
pcshow(clouds{4});
hold on;
pcshow(pctransform(clouds{3},currMotion2next));

%我们ICP，加利用上回运动
d=13;
m=14;
readyCloud1=pcdownsample(pcdenoise(clouds{d}),'gridAverage',gridstep);
readyCloud2=pcdownsample(pcdenoise(clouds{m}),'gridAverage',gridstep);
ns=createns(readyCloud2.Location,'nsmethod','kdtree');
motion=myTrimICP(ns,[readyCloud2.Location';ones(1,readyCloud2.Count)],[readyCloud1.Location';ones(1,readyCloud1.Count)],relativeMotion{m-1},50,0.35);
pcshow(clouds{m});
hold on;
pcshow(pctransform(clouds{d},affine3d(motion')));




%% 某两帧eig匹配
addpath('./flann/');
addpath('./estimateRigidTransform')
s=1;
res=1;
% ModelCloud=clouds{1};
% DataCloud=clouds{2};
mo=56;
da=mo+1;
ModelCloud=pointCloud(trclouds{mo}.Location);%clouds{234}
DataCloud=pointCloud(trclouds{da}.Location);%clouds{687}
gridStep= 0.4;%30
overlap=0.7;
tic
[tarDesp,tarSeed,tarNorm] = extractEig(ModelCloud,gridStep); 
[srcDesp,srcSeed,srcNorm] = extractEig(DataCloud,gridStep);
T = eigMatch(tarDesp,srcDesp,tarSeed,srcSeed,tarNorm,srcNorm,overlap,gridStep);
T = inv(T);
R0= T(1:3,1:3);
t0= T(1:3,4);
Model= ModelCloud.Location(1:res:end,:)';
Data= DataCloud.Location(1:res:end,:)';

[MSE,R,t] = TrICP(Model, Data, R0, t0, 100, overlap);
Motion=Rt2M(R,t);
Motion(1:3,4)=Motion(1:3,4).*s;

pcshow(pctransform(trclouds{da},affine3d(Motion')));
hold on;
pcshow(trclouds{mo});
% relativeMotionRe{mo}=Motion;
%% 展示某帧和第一帧配准
close all;
tar=3;
pcshow(clouds{1});
hold on;
pcshow(pctransform(clouds{tar},affine3d( Motion')));


% %% 某帧真值比对 
% load outside_GRT;
% tar =28;
% src=1;
% realMotion=GrtM{src}\GrtM{tar};   % inv(GrtM{src})*GrtM{tar}
% norm(realMotion(1:3,1:3)-motionInfo{1,1}(1:3,1:3),'fro')
% norm(realMotion(1:3,4)-motionInfo{1,1}(1:3,4).*30,2)
% 
% 

%% test CEll_fun
fixMotion=cellfun(@(x) {relativeMotion{x},x-1,x} , {2,3},'UniformOutput',false )
cellfun(@(x) {relativeMotion{x},x-1,x} , fixedPointCloudN,'UniformOutput',false );
cell2mat(fixMotion)
[historyAccMotion;fixMotion{1}]

%% 求两点距离
p1=[-1.262 0.6227]
p2=[0,0]
norm(p2-p1)

%% 点间关系拟合
x=historyCameraPosePair(:,2)-historyCameraPosePair(:,1);
y=historyCameraPosePair(:,4);
z=historyCameraPosePair(:,3);
reTool1=cftool(x,y,z+10);