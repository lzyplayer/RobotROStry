function rosPCCallback(~,massage)
%ROSPCCALLBACK 此处显示有关此函数的摘要
%   此处显示详细说明
% 
% massage
ptcloud = massage;
global pcObj
allInpoints=readXYZ(ptcloud);
% 分歧1   clean dirty points
% pointsFar=sqrt(allInpoints(:,1).^2+allInpoints(:,2).^2)>6;
% pointsGround=allInpoints(:,3)<-0.44;
% pointSelect=~(pointsFar | pointsGround);
% pcObj = pointCloud(allInpoints(pointSelect,:));

% 分歧2   raw
pcObj=pointCloud(allInpoints);

end

