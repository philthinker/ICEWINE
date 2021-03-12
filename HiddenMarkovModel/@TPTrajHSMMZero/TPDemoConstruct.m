function [Demo, frames] = TPDemoConstruct(obj,data,A,b)
%TPDemoConstruct Construct the TPDemo struct array by data, A and b. You
%can assign no arguments just for memory allocation.
%   data: D x N, data in world frame
%   A: D x D x F, orientation matrices of each frame
%   b: D x F, position vectors of each frame
%   --------------------------------------------------
%   Demo: TPDemo struct, the demo
%   frames: 1 x F frame struct array:
%   |   A: D x D, A matrix
%   |   b: D x 1, b vector
%   @TPTrajHSMMZero

if nargin == 1
    % Memory allocation
    Demo.A = [];
    Demo.b = [];
    Demo.data = [];
    Demo.TPData = [];
    frames.A = [];
    frames.b = [];
    frames = repmat(frames,[1,obj.F]);
    return;
end

D = size(data,1);
N = size(data,2);
F = size(A,3);
Demo = [];

% Original data
Demo.A = A;
Demo.b = b;
Demo.data = data;
Demo.TPData = zeros(D,F,N);
for f = 1:F
    for n = 1:N
        Demo.TPData(:,f,n) = A(:,:,f)' * (data(:,n) - b(:,f));
    end
end


end

