function rosPCCallback(~,massage)
%ROSPCCALLBACK �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
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

