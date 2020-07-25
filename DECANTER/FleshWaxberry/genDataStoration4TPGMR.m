function [query] = genDataStoration4TPGMR(queryVar,queryFrames_A,queryFrames_b)
%genDataStoration4TPGMR Generate data storation for TP-GMR given query and
%TPGMMOne object
%   queryVar: Din x N, query variables
%   queryFrames_A: D x D x F, query frame rotation matrix
%   queryFrames_b: D x F, query frame displacement
%   query:
%   |   dataIn: queryVar
%   |   queryFrames:
%   |   |   A: queryFrames_A
%   |   |   b: queryFrames_b
%   |   dataOut: Dout x N, TPGMR predictions
%   |   Sigma: Dout x Dout x N, TPGRM covariances

Dout = size(queryFrames_A,1)-1;
N = size(queryVar,2);
F = size(queryFrames_A,3);
query.dataIn = queryVar;
queryFrame = struct('A',zeros(Dout+1,Dout+1),'b',zeros(Dout+1,1));
query.queryFrames = repmat(queryFrame,[1,F]);
for i = 1:F
    query.queryFrames(i).A = queryFrames_A(:,:,i);
    query.queryFrames(i).b = queryFrames_b(:,i);
end
query.dataOut = zeros(Dout,N);
query.Sigma = zeros(Dout,Dout,N);

end

