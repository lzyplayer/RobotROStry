function [turnFlag,zAxis,planeAxis] = turnDectecion(currMotion,turnThreshold)
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
turnFlag=false;
turn=rotm2eul(currMotion(1:3,1:3),'XYZ');
zAxis=abs(turn(3));
if zAxis>=turnThreshold
    turnFlag=true;
end
planeAxis=max(abs(turn(1:2)));
end

