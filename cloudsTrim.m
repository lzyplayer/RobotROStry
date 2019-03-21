function [trimedCloud] = cloudsTrim(oriCloud)
%CLOUDSTRIM 此处显示有关此函数的摘要
%   此处显示详细说明
if ~iscell(oriCloud)
    xyzPoints = oriCloud.Location;
    selectPoints = xyzPoints((xyzPoints(:,3)>-0.35 & xyzPoints(:,3)<4.9),:);
    trimedCloud = pointCloud(selectPoints);
else 
    for i=1:length(oriCloud)
        xyzPoints = oriCloud{i}.Location;
    	selectPoints = xyzPoints((xyzPoints(:,3)>-0.35 & xyzPoints(:,3)<4.9),:);
        trimedCloud{i} = pointCloud(selectPoints);
    end
end
end

