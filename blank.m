% clc;clear;close all;
load('../ShunYuFullCloud.mat');
pcwrite(fullcloud,'sy_factory.pcd','Encoding','ascii');
curr_pc = pcread('sy_factory.pcd');
pcshow(curr_pc);




%%
ok = [clouds(1:87),clouds_temp(1:5),clouds(88:end-1)]
clouds = ok;

da

%%
obtainResult(clouds,MotionGlobal(1:60),false,mergeStep);

pcwrite(fullCloudCopy,'fullcloud.pcd','Encoding','ascii');
points2d = down3dto2d(fullCloudCopy,1,2);
cloud2d = pointCloud([points2d,zeros(length(points2d),1)]);
pcwrite(cloud2d,'gridCloud.pcd','Encoding','ascii');
pcshow(cloud2d);