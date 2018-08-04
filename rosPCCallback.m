function rosPCCallback(~,massage)
%ROSPCCALLBACK 此处显示有关此函数的摘要
%   此处显示详细说明
% 
% massage
ptcloud = massage;
global pcObj
allInpoints=readXYZ(ptcloud);
%clean dirty points
pointsFar=sqrt(allInpoints(:,1).^2+allInpoints(:,2).^2)>4.5;
pointsGround=allInpoints(:,3)<-0.65;
pointSelect=~(pointsFar | pointsGround);
pcObj = pointCloud(allInpoints(pointSelect,:));

end

