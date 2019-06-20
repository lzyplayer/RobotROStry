function [ Motion ,MSE ] = simpleRegWithEig( mapDescriptor,currDescriptor,ModelCloud ,DataCloud,overlap,gridStep,res,MSEthersold)
%MATCHFIX 此处显示有关此函数的摘要
%   此处显示详细说明


%     if(size(tarDesp,2)<100 || size(srcDesp,2)<100)
%         disp(['description points not enough! ' num2str(size(tarDesp,2)) ' ' num2str(size(srcDesp,2))])
%     end
%% if (size(mapDescriptor{1},2)<=size(currDescriptor{1},2))
if (size(mapDescriptor{1},2)>=size(currDescriptor{1},2))
    [T,rawMSE] = eigMatch_err(mapDescriptor{1},currDescriptor{1},mapDescriptor{2},currDescriptor{2},mapDescriptor{3},currDescriptor{3},overlap,gridStep);
    T = inv(T);
else
    [T,rawMSE] = eigMatch_err(currDescriptor{1},mapDescriptor{1},currDescriptor{2},mapDescriptor{2},currDescriptor{3},mapDescriptor{3},overlap,gridStep);

end
disp(['raw resgist with err: ' num2str(rawMSE)]);
R0 = T(1:3,1:3);
t0 = T(1:3,4);
Model= ModelCloud.Location(1:res:end,:)';
Data= DataCloud.Location(1:res:end,:)';
[MSE,R,t] = TrICP(Model, Data, R0, t0, 100, overlap);
if MSE>MSEthersold
    T=[];
    warning([' cannot get match result with err lower than: ' num2str(MSEthersold)
             ' currRawMSE: ' num2str(rawMSE)
             ' currMSE: ' num2str(MSE)]);
    return ;
end

Motion=Rt2M(R,t);

% Motion=Rt2M(R0,t0);
% fixFrameTotalTime=toc;
% disp(['fixtime is ' num2str(fixFrameTotalTime) 'seconds']);

end

