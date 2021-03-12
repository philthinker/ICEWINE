function [Demo, frames] = TPDemoConstruct_Dynamic(obj,data,A,b)
%TPDemoConstruct Construct the dynamic TPDemo struct array by position data, A and b. You
%can assign no arguments just for memory allocation.
%   data: DP x N, data in world frame
%   A: DP x DP x F, orientation matrices of each frame
%   b: DP x F, position vectors of each frame
%   --------------------------------------------------
%   Demo: TPDemo struct, the demo of dynamic data (D = DP*DD)
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

DP = size(data,1);
DD = obj.DD;
N = size(data,2);
F = size(A,3);
dt = obj.dt;
Demo = [];

% Original data
Demo.A = zeros(DP*DD, DP*DD, F);
Demo.b = zeros(DP*DD, F);
for f = 1:F
    Demo.A(:,:,f) = kron(eye(DD),A(:,:,f));
    Demo.b(:,f) = [b(:,f); zeros((DD-1)*DP,1)];
end
Demo.data = repmat(data,[DD,1]);
for i = 2:DD
    Demo.data((i-1)*DP+1: i*DP, :) = gradient(Demo.data((i-2)*DP+1:(i-1)*DP,:));
    Demo.data((i-1)*DP+1: i*DP, 1) = zeros(DP,1);
end
Demo.TPData = zeros(DP*DD,F,N);
for f = 1:F
    for n = 1:N
        Demo.TPData(:,f,n) = Demo.A(:,:,f)' * (Demo.data(:,n) - Demo.b(:,f));
    end
end


end

