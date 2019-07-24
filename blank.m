% clc;clear;close all;
load('../ShunYuFullCloud.mat');
pcwrite(fullcloud,'sy_factory.pcd','Encoding','ascii');
curr_pc = pcread('sy_factory.pcd');
pcshow(curr_pc);


pcwrite(fullcloud,'shunYuFactory.pcd','Encoding','ascii');
pcwrite(pcObj,'shunYu1.pcd','Encoding','ascii');

%%
ok = [clouds(1:87),clouds_temp(1:5),clouds(88:end-1)]
clouds = ok;

%da

%%
obtainResult(clouds,MotionGlobal(1:60),false,mergeStep);

pcwrite(fullCloudCopy,'fullcloud.pcd','Encoding','ascii');
points2d = down3dto2d(fullCloudCopy,1,2);
cloud2d = pointCloud([points2d,zeros(length(points2d),1)]);
pcwrite(cloud2d,'gridCloud.pcd','Encoding','ascii');
pcshow(cloud2d);
%%
dfcloud2d = down3dto2d(full_cloud,0,4);
dccloud2d = down3dto2d(pcObj,0,4);
downFullCloud = pcdownsample( dfcloud2d,'gridAverage',0.01);
downcurrCloud = pcdownsample( dccloud2d,'gridAverage',0.01);
pcwrite(downFullCloud,'shunYuFactory.pcd','Encoding','ascii');
pcwrite(downcurrCloud,'shunYu1.pcd','Encoding','ascii');
%% write pcd

points2d = down3dto2d(fullcloud,0,4);
ecloud  = pointCloud([points2d,zeros(size(points2d,1),1)]);
downecloud = pcdownsample(ecloud,'gridAverage',0.03);
pcwrite(downecloud,'shunYuFactory.pcd','Encoding','ascii');

points2d = down3dto2d(clouds{1},0,4);
ecloud  = pointCloud([points2d,zeros(size(points2d,1),1)]);
downecloud = pcdownsample(ecloud,'gridAverage',0.03);
pcwrite(downecloud,'shunYu1.pcd','Encoding','ascii');


% pcwrite(pcdownsample(clouds{1},'gridAverage',0.05),'shunyu1.pcd','Encoding','ascii');
