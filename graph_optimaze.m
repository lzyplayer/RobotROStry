% import
posegraph = robotics.PoseGraph3D;
infoM= [1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1];
for i=2:length(centerClouds)
    relativeM =  MotionGlobal{i-1}\MotionGlobal{i};
    transformation = [relativeM(1:3,4)',rotm2quat(relativeM(1:3,1:3))];
    addRelativePose(posegraph,transformation,infoM,i-1,i);
end
transform  = [rM22_38(1:3,4)',rotm2quat(rM22_38(1:3,1:3))];
addRelativePose(posegraph,transform ,infoM*100,22,38);
transform  = [rM22_39(1:3,4)',rotm2quat(rM22_39(1:3,1:3))];
addRelativePose(posegraph,transform ,infoM*100,22,39);
transform  = [rM23_38(1:3,4)',rotm2quat(rM23_38(1:3,1:3))];
addRelativePose(posegraph,transform ,infoM*100,23,38);
%% g2o
    show(posegraph);
    disp(posegraph);
    optimizePoseGraph(posegraph,"g2o-levenberg-marquardt",'VerboseOutput','on'); %"builtin-trust-region" (default) | "g2o-levenberg-marquardt"
    show(posegraph);
    disp(posegraph);
    
        %% merge clouds
        
        downsampleGrid=0.01;
        clouds = centerClouds;
    fullcloud = clouds{1};
    nodes= posegraph.nodes;
    for i= 2:length(nodes)
        tranmatrix = eye(4);
        tranmatrix(1:3,:) = [quat2rotm(nodes(i,4:7)),nodes(i,1:3)'];
        fullcloud=pcmerge(fullcloud,pctransform(clouds{i},affine3d(tranmatrix')),downsampleGrid);
        MotionalGlobal{i} = tranmatrix;
    end
    MotionalGlobal{1} =eye(4);
    pcshow(fullcloud);
    N=length(clouds);
    close all;
    routeDisplay(MotionGlobal,'r-o',false,[18,19]);
    obtainResult(clouds(1:end),MotionGlobal(1:N),false,downsampleGrid);