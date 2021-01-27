function [TPGMRData] = TPGMRDataConstruct(obj, query, A, b)
%TPGMRDataConstruct Construct the portable data struct for TP-GMR. You can
%assign no arguments just for memory allocation.
%   query: 1 x N, the query
%   A: D x D x F, the A matrices
%   b: D x F, the b vectors
%   -------------------------------------------------
%   TPGMRData: struct:
%   |   query: 1 x N, the query
%   |   frames: 1 x F struct array
%   |   |   A: D x D, the A matrix
%   |   |   b: D x 1, the b vector
%   |   data: Dout x N, the  output data storage ([ ])
%   |   Sigma: Dout x Dout x N, the output covariances  storage ([ ])
%   @TPGMMOne

F = obj.nFrame;

if nargin == 1
    % Memory allocation
    TPGMRData.query = [];
    frames.A = [];
    frames.b = [];
    TPGMRData.frames = repmat(frames,[1,F]);
    TPGMRData.data = [];
    TPGMRData.Sigma = [];
    return;
end

TPGMRData.query = query;
frames.A = [];
frames.b = [];
frames = repmat(frames,[1,F]);
for i = 1:F
    frames(i).A = A(:,:,i);
    frames(i).b = b(:,i);
end
TPGMRData.frames = frames;
TPGMRData.data = [];
TPGMRData.Sigma = [];

end

