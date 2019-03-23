clc;clear;close all;
rosshutdown;
rosinit('192.168.67.129');
rosSuber=rossubscriber('/velodyne_points',@rosPCCallback);
global pcObj
% if ~isempty(pcObj)
%     pcshow(pcObj);
% end
while isempty(pcObj)
    pause(0.1);
    disp('cannot get pointCloud')
end

commond='';
clouds{1}=pcObj;
% fig=figure('Position',[55 173 590 696]);
% pcshow(clouds{1});
% hold off
% reScatter = reAxes.Children;
i=1;
while  isempty(commond) %&& commond~='q'
    commond=input('please input commond(enter for next): \n','s');
    i=i+1;
    clouds{i}=pcObj;
    disp(['cloud ' int2str(i) ' stored!' ])
%     pcshow(clouds{i});
end
name='scienceBuild_reGet.mat';
save(name,'clouds');
disp(['clouds saved as ' name '!' ])

rosshutdown
clear('rosSuber');


% for i=1:20
%     input('please input commond:\n','s');
%     clouds{i}=pcObj;
% end
% figure;axis on;
% while true
%     pcshow(pcObj);
%     pause(0.5);
% end

%% try Timer bad performence
% t = timer;
% t.ExecutionMode='fixedRate';
% t.Period=0.25;
% t.TimerFcn =@(myTimerObj, thisEvent)pcshow(pcObj);
% start(t);
% delete(t);% stop timer