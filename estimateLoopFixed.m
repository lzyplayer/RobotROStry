function [ cameraPosePair ,flag] = estimateLoopFixed( globalCameraPosition,cameraPosePair,LoopDectNum ,flag,maxDis)
%ESTIMATELOOP �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
addNum=0;
curr=length(globalCameraPosition);
currPair=[];


for i=1:curr-LoopDectNum

    distance=norm(globalCameraPosition(curr,1:2)-globalCameraPosition(i,1:2));%     maxDis=0.345*(curr-i)-19.4;         %sqrt(curr-lastLoopNum)*23;
    if(distance)< maxDis
        pairInfo=[i curr distance];
        currPair=[currPair;pairInfo];
        if(flag == 0)
            flag=1;
        end
    end
end
    if (~isempty(currPair))
    sortedPair=sortrows(currPair,3);
    if(size(sortedPair,1)>4)
        cameraPosePair=[cameraPosePair;sortedPair(1:4,:)];
    else
        cameraPosePair=[cameraPosePair;sortedPair];
    end
    end
end

