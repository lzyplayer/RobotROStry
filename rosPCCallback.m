function rosPCCallback(~,massage)
%ROSPCCALLBACK �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
% 
% massage
ptcloud = massage;
global pcObj
allInpoints=readXYZ(ptcloud);
% ����1   clean dirty points
% pointsFar=sqrt(allInpoints(:,1).^2+allInpoints(:,2).^2)>6;
% pointsGround=allInpoints(:,3)<-0.44;
% pointSelect=~(pointsFar | pointsGround);
% pcObj = pointCloud(allInpoints(pointSelect,:));

% ����2   raw
pcObj=pointCloud(allInpoints);

end

