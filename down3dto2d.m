function [selectPoints] = down3dto2d(originalCloud , low ,high ,center,radius)
%% put cloud 2 2d
%   �˴���ʾ��ϸ˵��
if length( originalCloud)==1
     originalCloud=originalCloud.Location; 
end
GroundSkyPointer = originalCloud(:,3)<low | originalCloud(:,3)>high;
selectPoints = originalCloud(~(GroundSkyPointer ),:);
selectPoints = selectPoints(:,1:2);


if nargin == 5
%%
Idx = rangesearch(selectPoints,center,radius);
selectPoints = selectPoints(Idx{1},:);

end

end

