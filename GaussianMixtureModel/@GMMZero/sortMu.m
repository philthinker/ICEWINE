function [obj] = sortMu(obj,index)
%sortMu Sort the Mu of the GMM by dimension index
%   index: scalar, the dimension to be sorted along
%   @GMMZero

if index <= obj.nKernel && index >= 1
    [~,I] = sort(obj.Mu(:,index),1);
    obj.Mu = obj.Mu(I,:);
    obj.Sigma = obj.Sigma(:,:,I);
    obj.Prior = obj.Prior(I,:);
end
    

end

