function [demo] = dataReconstruct(obj,A,b,data0)
%dataReconstruct Reconstruct the demo for TP-GMM
%   A: D x D x F, rotation matrix
%   b: D x F, position
%   data0: N x D, the demo data
%   @TPGMM

F = size(A,3);
D = size(A,1);
N = size(data0,1);
demo.A = A;
demo.b = b;
demo.data = data0;
demo.TPData = zeros(D,F,N);
for i = 1:F
    demo.TPData(:,i,:) = permute(A(:,:,i)\(data0'-b(:,i)),[1,3,2]);
end

end

