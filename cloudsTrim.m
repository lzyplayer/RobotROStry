function [trimedCloud] = cloudsTrim(oriCloud)
%CLOUDSTRIM �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
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

