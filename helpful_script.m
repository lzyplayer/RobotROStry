%% ��������������
hold on;
currSeedPointCloud = pointCloud(currSeed');
transferedCloud = pctransform(currSeedPointCloud,affine3d(T'));
transLocation = transferedCloud.Location;
% pcshow(transferedCloud.Location,single([0 1 1]),'MarkerSize',30);%
plot3(transLocation(:,1),transLocation(:,2),transLocation(:,3),'c*','MarkerSize',16);
currAxes.Color=[1,1,1];
%% ����һ��ȫ�ֶ�λ��������㲢����?
hold on;
M = size(match_tarSeed,2);
traned_match_tarSeed = T*[match_tarSeed;ones(1,M)];

for i=1:size(match_tarSeed,2)
    line([match_srcSeed(1,i);traned_match_tarSeed(1,i)],[match_srcSeed(2,i);traned_match_tarSeed(2,i)],[match_srcSeed(3,i);traned_match_tarSeed(3,i)],'Color','red','Marker','o','MarkerSize',10);  
    
end
view(3)
plot3(match_srcSeed(1,:)',match_srcSeed(2,:)',match_srcSeed(3,:)','*')
plot3(match_srcSeed(1,1)',match_srcSeed(2,1)',match_srcSeed(3,1)','*','MarkerSize',20)
%% �ҵ�ƥ��Ե����
MatchSeedIdx=[];
for i=1:size(match_srcSeed,2)
    MatchSeedIdx=[MatchSeedIdx,[find(mapSeed(1,:)==match_srcSeed(1,i));find(currSeed(1,:)==match_tarSeed(1,i))]];
%     mapMatchSeedIdx=[mapMatchSeedIdx,find(mapSeed(1,:)==match_srcSeed(1,i)) ];
%     currMatchSeedIdx=[currMatchSeedIdx,];
    
end


%% anime show
axes = pcshow(clouds{1});
cscatter = axes.Children;
for i=87:88
    currPoints = clouds{i}.Location;
    cscatter.XData=currPoints(:,1);
    cscatter.YData=currPoints(:,2);
    cscatter.ZData=currPoints(:,3);
    cscatter.CData=currPoints(:,3);
    drawnow();
    pause(1);
end
%% �������?
norm(center.Position(1:2)-flow_point.Position(1:2))


p1=MotionGlobal{20}(1:2,4);
p2=MotionGlobal{68}(1:2,4);
norm(p1-p2)
%% ����չʾ
hold off
for i=145:length(clouds)
    pcshow(clouds{i});
end

return 
%% ��ֵ����չʾ
load hannover2_MZ.mat
load hannover2_GrtM_z.mat
routeDisplay(GrtM,'g-d',false,[48,55]);%(1:182)(1:532)799,490
obtainResult(clouds,GrtM(1:100),true);

%% չʾ����˶�ĳ��?
tosee=57;
% for currsee=tosee:tosee+8
figure;
pcshow(trclouds{tosee-1});hold on;
pcshow(pctransform(trclouds{tosee},affine3d(relativeMotionRe{tosee}')));%historyAccMotion{pairnum}'
% end
%% temp
figure;
tosee=84;
% for currsee=tosee:tosee+8
figure;
pcshow(clouds{tosee-1});hold on;
pcshow(pctransform(clouds{tosee},affine3d(accMotion{97,1}')));%historyAccMotion{pairnum}'
% end

%% ȫ�ַ���չ�֣���·�������?
% ori=[0 0 0 ;100 100 100 ];%ȫ�ֹ۲ⷽ��
ori=[0 0 0 ;0.01 0.01 0.01 ];

for i=1:length(MotionGlobal)
    currOri=MotionGlobal{i}*[ori';ones(1,size(ori,1))];
    eachOri{i}=currOri(1:3,:)';
    plot3(eachOri{i}(:,1),eachOri{i}(:,2),eachOri{i}(:,3),'r-o')
    hold on
end

%% չʾ·��ͼ

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

% %test�¹����ڵ���
% scanTest=load('./data/scanTest.3d');
% cloudTest=pointCloud(scanTest(:,1:3));
% pcshow(cloudTest);
% 

%% ���������?
tar = [-20.84 6.196];
sta = [-18.04 5.256];
addt= tar-sta
%% ��ֵ���?
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

%% ��ȡչʾ����
% datapath='./data/red_room/map2.ply';
% clouds=pcread(datapath);
% 
% pcshow(clouds);

%% չʾ��׼
% s=30;
close all
figure('position',[-1439 76 1440 823])
hold on;

for i=114:115
    currMotion=MotionGlobal{i};
%     curMotion(1:3,4)=curMotion(1:3,4)./s;
    pcshow(pctransform( trclouds{i},affine3d(currMotion')));
    
end
%% չʾ��׼
close all
mergeGridStep=0.025;
figure('position',[-1439 76 1440 823])
hold on;
obtainResult(trclouds,MotionGlobalRe(1:90),false,mergeGridStep);
%% ����˶�չ�?
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


%% ����
% for i=1:length(cloudsCopy)
% [clearClouds{i},inlinerOnes{i},outlierOnes{i}]=pcdenoise(cloudsCopy{i});
% end

%% չʾ����
% s=30;
% for i=1:length(cloudsCopy)
%     curMotion=p(i).M;
%     curMotion(1:3,4)=curMotion(1:3,4)./s;
%     pcshow(pctransform (pointCloud(cloudsCopy{i}.Location(outlierOnes{i},:)),affine3d(curMotion')));
%     hold on
% end
