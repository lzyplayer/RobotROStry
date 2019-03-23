function [trimedCloud] = cloudsTrim(oriCloud)
%CLOUDSTRIM 此处显示有关此函数的摘要
%   此处显示详细说明
if ~iscell(oriCloud)
    xyzPoints = oriCloud.Location;
    GroundSkyPointer = xyzPoints(:,3)<-0.427 | xyzPoints(:,3)>4.8;
    carPointer = xyzPoints(:,1)<-0.54 & xyzPoints(:,1)>-1 & xyzPoints(:,2)<0.29 & xyzPoints(:,2)>-0.43;
    selectPoints = xyzPoints(~(GroundSkyPointer | carPointer),:);
    trimedCloud = pcdenoise( pointCloud(selectPoints));
else
    for i=1:length(oriCloud)
        xyzPoints = oriCloud{i}.Location;
        GroundSkyPointer = xyzPoints(:,3)<-0.427 | xyzPoints(:,3)>4.8;
        carPointer = xyzPoints(:,1)<-0.54 & xyzPoints(:,1)>-1 & xyzPoints(:,2)<0.29 & xyzPoints(:,2)>-0.43;
        selectPoints = xyzPoints(~(GroundSkyPointer | carPointer),:);
        trimedCloud{i} = pcdenoise( pointCloud(selectPoints));
        
    end
end
end

