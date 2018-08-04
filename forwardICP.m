function [ relativeMotion,MSE] = forwardICP( ns,i,dowmSampleClouds,clouds,ICPthreashold,overlap )
%FORWARDICP �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
 dowmSampleClouds{i}=pcdownsample(pcdenoise(clouds{i}),'gridAverage',icpGridStep);
%     [currMotion2next]=pcregrigid(dowmSampleClouds{i-1},dowmSampleClouds{i},'Tolerance',[0.01/s,0.009]);
    ns{i-1}=createns(dowmSampleClouds{i-1}.Location,'nsmethod','kdtree');
    IcpModel=[dowmSampleClouds{i-1}.Location';ones(1,dowmSampleClouds{i-1}.Count)];
    IcpData=[dowmSampleClouds{i}.Location';ones(1,dowmSampleClouds{i}.Count)];
    [relativeMotion{i},MSE(i,1)]=myTrimICP(ns{i-1},IcpModel,IcpData,relativeMotion{i-1},ICPthreashold,overlap);

end

