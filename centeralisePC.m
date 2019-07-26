clc;clear;close all;
load wuhan_factory_withoutground.mat
addpath('./flann/');
addpath('./estimateRigidTransform');


gridStep=2;
downsamplegrid = 0.05;
midlap = 5;
res=1;
MSEthersold=0.4;
overlap=0.6;
goodMSEthreshold=0.01;
%%
trclouds = pcTrim(objects,100,[-100 100],downsamplegrid);
%%
N=length(objects);
for i=18%0:N/midlap-1
    centerClouds{i+1} = accmulatePC(trclouds,i*midlap+1,midlap,overlap,gridStep,res,MSEthersold,downsamplegrid,goodMSEthreshold);
    pcshow(centerClouds{i+1});
%     input("next")
end

%  centerClouds{i+1}=fullcloudCopy;