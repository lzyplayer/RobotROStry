% clc;clear;close all;
load('../ShunYuFullCloud.mat');
pcwrite(fullcloud,'sy_factory.pcd','Encoding','ascii');
curr_pc = pcread('sy_factory.pcd');
pcshow(curr_pc);




%%
ok = [clouds(1:87),clouds_temp(1:5),clouds(88:end-1)]
clouds = ok;