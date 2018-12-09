function [ ] = obtainResult( clouds , motion,new,mergeStep)
%OBTAINRESULT �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
if new
    figure;
end
hold on;
fullCloud=clouds{1};
for i=1:length(motion)
    currMotion=motion{i};
    Euler=rotm2eul(currMotion(1:3,1:3),'XYZ');
    
    %����1   ȫչʾ
%     zEuler=Euler;
    %����2   ��z��
        zEuler=[0 , 0, Euler(3)];
    
    conRot=eul2rotm( zEuler,'XYZ');
    conTran=[currMotion(1:2,4);0];
    currMotion=Rt2M(conRot,conTran);
    
    if (isRigid(affine3d(currMotion')))
        fullCloud=pcmerge(fullCloud,pctransform(clouds{i},affine3d(currMotion')),mergeStep);
    else
        Location=clouds{i}.Location';
        TLocation=currMotion*[Location;ones(1,size(Location,2))];
        currPointCloud=pointCloud(TLocation(1:3,:)');
        fullCloud=pcmerge(fullCloud,currPointCloud,mergeStep);
        
    end
end
pcshow(fullCloud);
% figure;
% for i=238:239
%     pcshow(pctransform(clouds{i},affine3d(motion{i}')));
%     hold on;
% end

