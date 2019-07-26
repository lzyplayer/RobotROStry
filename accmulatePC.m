function [fullcloud] = accmulatePC(clouds,start,clength,overlap,gridStep,res,MSEthersold,downsampleGrid,goodMSEthreshold)
%UNTITLED ¦
%   

    %% with posegraph
    posegraph = robotics.PoseGraph3D;
    fullcloud = clouds{start};
    last=start-1+clength;
    for i = start+1:last
        [ Motion ,MSE ,fixtime] = matchFix(fullcloud,clouds{i},overlap,gridStep,res,i,MSEthersold);
        fprintf('frame %i\t MSE %f\t fixtime %f\n',i,MSE,fixtime)
         if(isempty(Motion))
             warning("cannot match %i %i ",i,start);
            transform=[0,0,0,1,0,0,0];
            infoM  = [1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1]*0.001;
            addRelativePose(posegraph,transform,infoM,i-start+1,1);
            continue;
         end
        
        if(MSE>goodMSEthreshold)
            warning("abandon match on %i %i ",i,start);
        else
            fullcloud = pcmerge(fullcloud,pctransform(clouds{i},affine3d(Motion')),downsampleGrid);
        end
        transform  = [Motion(1:3,4)',rotm2quat(Motion(1:3,1:3))];
        infoM  = [1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1]* (1/MSE) ;%
        addRelativePose(posegraph,transform,infoM,i-start+1,1);

    end  
    fullcloud = clouds{last};
    for i = last-1:-1:start
        [ Motion ,MSE ,fixtime] = matchFix(fullcloud,clouds{i},overlap,gridStep,res,i,MSEthersold);
        fprintf('frame %i\t MSE %f\t fixtime %f\n',i,MSE,fixtime)
        if(isempty( Motion)|| MSE>goodMSEthreshold)
            warning("abandon match on %i %i",i,last);
            continue;
        end
        fullcloud = pcmerge(fullcloud,pctransform(clouds{i},affine3d(Motion')),downsampleGrid);
        transform  = [Motion(1:3,4)',rotm2quat(Motion(1:3,1:3))];
        infoM  = [1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1]* (1/MSE*10);
        addRelativePose(posegraph,transform,infoM,i-start+1,clength);
    end  
%     show(posegraph);
%     disp(posegraph);
    optimizePoseGraph(posegraph,"g2o-levenberg-marquardt",'VerboseOutput','on'); %"builtin-trust-region" (default) | "g2o-levenberg-marquardt"
%     show(posegraph);
%     disp(posegraph);
    %% merge clouds
    fullcloud = clouds{start};
    nodes= posegraph.nodes;
    for i= start+1:last
        tranmatrix = eye(4);
        tranmatrix(1:3,:) = [quat2rotm(nodes(i-start+1,4:7)),nodes(i-start+1,1:3)'];
        fullcloud=pcmerge(fullcloud,pctransform(clouds{i},affine3d((inv(tranmatrix))')),downsampleGrid);
    end
    
    
end

