function [trimmedClouds] = pcTrim(clouds,distance,zlimit,gridstep)
%PCTRIM �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
for i=1:length(clouds)
    allInpoints = clouds{i}.Location;
    pointsFar = sqrt(allInpoints(:,1).^2+allInpoints(:,2).^2)>distance;
    pointsGround = (allInpoints(:,3)<zlimit(1)) | (allInpoints(:,3)>zlimit(2));
    pointSelect = ~(pointsFar | pointsGround);
    trimmedClouds{i} = pcdownsample(pcdenoise(pointCloud(allInpoints(pointSelect,:))),'gridAverage',gridstep);
end
end

