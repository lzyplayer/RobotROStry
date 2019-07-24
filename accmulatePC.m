function [fullcloud] = accmulatePC(clouds,start,clength,overlap,gridStep,res,MSEthersold,downsampleGrid)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
    fullcloud = clouds{start};
    for i = start+1:start-1+clength
        [ Motion ,MSE ,fixtime] = matchFix(fullcloud,clouds{i},overlap,gridStep,res,i,MSEthersold);
        fprintf('frame %i\t MSE %f\t fixtime %f\n',i,MSE,fixtime)
        fullcloud = pcmerge(fullcloud,pctransform(clouds{i},affine3d(Motion')),downsampleGrid);
    end
    
end

