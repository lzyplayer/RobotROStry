p=zeros(length(trclouds),1);
for i=1:length(trclouds)
    volume=abs((trclouds{i}.XLimits(2)-trclouds{i}.XLimits(1))*(trclouds{i}.YLimits(2)-trclouds{i}.YLimits(1))*(trclouds{i}.ZLimits(2)-trclouds{i}.ZLimits(1)));
    p(i)=trclouds{i}.Count/volume;
    
end
aveP=mean(p);
normalP=1.2980e+07;
normalG=0.01;
value=nthroot(((normalP*(normalG^3))/(aveP)),3)
