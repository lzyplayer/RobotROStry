%% ����չʾ
hold off
for i=1:length(clouds)
    pcshow(clouds{i});
end
%% ��ֵ���
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

for i=1:length(clouds)
    curMotion=p(i).M;
    curMotion(1:3,4)=curMotion(1:3,4)./s;
    pcshow(pctransform( colorful_clouds{i},affine3d(curMotion')));
    hold on;
end

for i=1:2
%     curMotion=p(i).M;
%     curMotion(1:3,4)=curMotion(1:3,4)./s;
    pcshow(pcdenoise(clouds{2}));
    hold on;
end
  pcshow(pcdenoise(pctransform(  clouds{2}, affine3d(Motion'))));
  hold on;


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
