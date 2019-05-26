%% 两点距离
p1=MotionGlobal{20}(1:2,4);
p2=MotionGlobal{68}(1:2,4);
norm(p1-p2)
%% 依次展示
hold off
for i=145:length(clouds)
    pcshow(clouds{i});
end

return 
%% 真值环境展示
load hannover2_MZ.mat
load hannover2_GrtM_z.mat
routeDisplay(GrtM,'g-d',false,[48,55]);%(1:182)(1:532)799,490
obtainResult(clouds,GrtM(1:100),true);

%% 展示相对运动某对
tosee=57;
% for currsee=tosee:tosee+8
figure;
pcshow(trclouds{tosee-1});hold on;
pcshow(pctransform(trclouds{tosee},affine3d(relativeMotionRe{tosee}')));%historyAccMotion{pairnum}'
% end

%% 全局方向展现，和路径结合用
% ori=[0 0 0 ;100 100 100 ];%全局观测方向
ori=[0 0 0 ;0.01 0.01 0.01 ];

for i=1:length(MotionGlobal)
    currOri=MotionGlobal{i}*[ori';ones(1,size(ori,1))];
    eachOri{i}=currOri(1:3,:)';
    plot3(eachOri{i}(:,1),eachOri{i}(:,2),eachOri{i}(:,3),'r-o')
    hold on
end

%% 展示路径图

route=[];
for p=1:length(MotionGlobal)
    route=[route; MotionGlobal{p}(1:3,4)'];
end
plot3(route(:,1),route(:,2),route(:,3),'-*');
xlabel('x');
ylabel('y');
zlabel('z');
% axis([-3000 3000 -3000 3000 -3000 3000 ]);
axis([-0.2 0.2 -0.2 0.2 -0.2 0.2 ]);

% %test德国室内点云
% scanTest=load('./data/scanTest.3d');
% cloudTest=pointCloud(scanTest(:,1:3));
% pcshow(cloudTest);
% 

%% 两点差距计算
tar = [-20.84 6.196];
sta = [-18.04 5.256];
addt= tar-sta
%% 真值误差
%scannum=length(clouds);
% for i=1:scannum
%     Data_{i}=clouds{i}.Location;
%     
% end
% 
% RotErr=0;
% TranErr=0;
% scannum=length(GrtR);
% 
% for i=1:scannum
%     RotErr=norm((p(i).M(1:3,1:3)-GrtR{i}),'fro')+RotErr;
%     TranErr=norm((p(i).M(1:3,4)-GrtT{i}),2)+TranErr;
%     
%    
% end
% RotErr=RotErr/scannum
% TranErr=TranErr/scannum
%  RotLogErr=log(RotErr)
% 
% 


% clc;
% close all;
% clear;

%% 读取展示单个
% datapath='./data/red_room/map2.ply';
% clouds=pcread(datapath);
% 
% pcshow(clouds);

%% 展示配准
% s=30;
close all
figure('position',[-1439 76 1440 823])
hold on;

for i=114:115
    currMotion=MotionGlobal{i};
%     curMotion(1:3,4)=curMotion(1:3,4)./s;
    pcshow(pctransform( trclouds{i},affine3d(currMotion')));
    
end
%% 展示配准
close all
mergeGridStep=0.025;
figure('position',[-1439 76 1440 823])
hold on;
obtainResult(trclouds,MotionGlobalRe(1:90),false,mergeGridStep);
%% 相对运动展示
close all
figure('position',[-1439 76 1440 823])
mo=118;
pcshow(clouds{mo});
hold on
i=mo+1;
pcshow(pctransform( clouds{i},affine3d(relativeMotionRe{i}')));
%     curMotion=p(i).M;
%     curMotion(1:3,4)=curMotion(1:3,4)./s;
% hold on;

%   pcshow(pcdenoise(pctransform(  clouds{2}, affine3d(Motion'))));
%   hold on;


%% 降噪
% for i=1:length(cloudsCopy)
% [clearClouds{i},inlinerOnes{i},outlierOnes{i}]=pcdenoise(cloudsCopy{i});
% end

%% 展示噪声
% s=30;
% for i=1:length(cloudsCopy)
%     curMotion=p(i).M;
%     curMotion(1:3,4)=curMotion(1:3,4)./s;
%     pcshow(pctransform (pointCloud(cloudsCopy{i}.Location(outlierOnes{i},:)),affine3d(curMotion')));
%     hold on
% end
