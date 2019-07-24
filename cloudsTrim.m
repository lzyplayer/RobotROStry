function [trimedCloud] = cloudsTrim(oriCloud)
%CLOUDSTRIM �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��

    xyzPoints = oriCloud.Location;
    GroundSkyPointer = xyzPoints(:,3)<0|xyzPoints(:,3)>4 ;
    selectPoints = xyzPoints(~(GroundSkyPointer ),:);
    trimedCloud = pcdenoise( pointCloud(selectPoints));

    

% if ~iscell(oriCloud)
%     xyzPoints = oriCloud.Location;
%     GroundSkyPointer = xyzPoints(:,3)<-0.34 | xyzPoints(:,3)>4.887;
%     carPointer = xyzPoints(:,1)<-0.54 & xyzPoints(:,1)>-1 & xyzPoints(:,2)<0.29 & xyzPoints(:,2)>-0.43;
%     selectPoints = xyzPoints(~(GroundSkyPointer | carPointer),:);
%     trimedCloud = pcdenoise( pointCloud(selectPoints));
% else
%     for i=1:length(oriCloud)
%         xyzPoints = oriCloud{i}.Location;
%         GroundSkyPointer = xyzPoints(:,3)<-0.34 | xyzPoints(:,3)>4.887;%-0.427 4.8
%         carPointer = xyzPoints(:,1)<-0.54 & xyzPoints(:,1)>-1 & xyzPoints(:,2)<0.29 & xyzPoints(:,2)>-0.43;
%         selectPoints = xyzPoints(~(GroundSkyPointer | carPointer),:);
%         trimedCloud{i} = pcdenoise( pointCloud(selectPoints));
%         
%     end
%     
% end
end

