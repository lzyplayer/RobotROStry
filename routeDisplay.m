function  routeDisplay( MotionGlobal ,type ,newFigure,specialshow)
%ROUTEDISPLAY �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
if(newFigure)
    figure;
end
% route=[];
% for p=1:length(MotionGlobal)
%     route=[route; MotionGlobal{p}(1:3,4)'];
% end
% plot(route(:,1),route(:,2),type);
% xlabel('x');
% ylabel('y');
% hold on;

route=[];
if(iscell(MotionGlobal))
    for p=1:length(MotionGlobal)
        route=[route; MotionGlobal{p}(1:3,4)'];
    end
else
    route=MotionGlobal;
end
% ����1---��ά
plot3(route(:,1),route(:,2),route(:,3),type);
xlabel('x');
ylabel('y');
zlabel('z');
hold on;
axis equal;
plot3(route(specialshow,1),route(specialshow,2),route(specialshow,3),'dk','MarkerSize',10);
%����2---��ά
% plot3(route(:,1),route(:,2),zeros(size(route,1),1),type);
% xlabel('x');
% ylabel('y');
% zlabel('z');
% hold on;
% axis equal;
% plot3(route(specialshow,1),route(specialshow,2),zeros(size(route,1),1),'dk','MarkerSize',10);





end

