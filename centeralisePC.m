load wuhan_factory_withoutground.mat
gridStep=0.7;
downsamplegrid = 0.02;
midlap = 5;
res=1;
MSEthersold=20;
overlap=0.8;
%%
trclouds = pcTrim(clouds,100,[-100 100],downsamplegrid);
%%
N=length(clouds);
centerClouds={};
for i=0:N/midlap-1
    centerClouds{i+1} = accmulatePC(trclouds,i*midlap+1,midlap,overlap,gridStep,res,MSEthersold,downsamplegrid);
    pcshow(centerClouds{i+1});
end