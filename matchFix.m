function [ Motion ,MSE ,fixtime] = matchFix( ModelCloud,DataCloud ,overlap,gridStep,res,curr,MSEthersold)
%MATCHFIX �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
tic
% gridStep=0.04;
% overlap=0.35;
% res=10;
MSE=0;
% [tarDesp,tarSeed,tarNorm] = extractEig(ModelCloud,gridStep);
% [srcDesp,srcSeed,srcNorm] = extractEig(DataCloud,gridStep);

% if(size(tarDesp,2)<size(srcDesp,2))
%     T = eigMatch(srcDesp,tarDesp,srcSeed,tarSeed,srcNorm,tarNorm,overlap,gridStep);
% else
%     T = eigMatch(tarDesp,srcDesp,tarSeed,srcSeed,tarNorm,srcNorm,overlap,gridStep);
%     T = inv(T);
% end
fixtime=0;T=[];
while(isempty(T)&&fixtime<5)
    
    [tarDesp,tarSeed,tarNorm] = extractEig(ModelCloud,(1-fixtime/10)*gridStep);
    tic;
    [srcDesp,srcSeed,srcNorm] = extractEig(DataCloud,(1-fixtime/10)*gridStep);
%     if(size(tarDesp,2)<100 || size(srcDesp,2)<100)
%         disp(['description points not enough! ' num2str(size(tarDesp,2)) ' ' num2str(size(srcDesp,2))])
%     end
    if (size(tarDesp,2)>=size(srcDesp,2))
        T = eigMatch(tarDesp,srcDesp,tarSeed,srcSeed,tarNorm,srcNorm,overlap,(1-fixtime/10)*gridStep);
        T = inv(T);
    else
        T = eigMatch(srcDesp,tarDesp,srcSeed,tarSeed,srcNorm,tarNorm,overlap,(1-fixtime/10)*gridStep);
    
    end
    fixtime=fixtime+1;
    if isempty(T)
        
        continue
    end
    R0= T(1:3,1:3);
    t0= T(1:3,4);
    Model= ModelCloud.Location(1:res:end,:)';
    Data= DataCloud.Location(1:res:end,:)';
    [MSE,R,t] = TrICP(Model, Data, R0, t0, 200, overlap);
    if MSE>MSEthersold
        T=[];
    end
end
if (isempty(T))
    Motion=[];
    warning([' cannot match cloud ' num2str(curr) ' with prev']);
    return ;
end
% R0= T(1:3,1:3);
% t0= T(1:3,4);
% Model= ModelCloud.Location(1:res:end,:)';
% Data= DataCloud.Location(1:res:end,:)';
% %
% [MSE,R,t] = TrICP(Model, Data, R0, t0, 100, overlap);
if(MSE>MSEthersold)
%     Motion=[];
    warning(['bad match alert: ' num2str(curr) ' with ' num2str(curr-1)]);
%     return;
end
Motion=Rt2M(R,t);
% Motion=Rt2M(R0,t0);
% fixFrameTotalTime=toc;
% disp(['fixtime is ' num2str(fixFrameTotalTime) 'seconds']);

end

