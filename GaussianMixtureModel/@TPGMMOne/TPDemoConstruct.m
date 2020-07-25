function [Demo] = TPDemoConstruct(obj,data,A,b,tmpTime)
%TPDemoConstruct Construct the TPDemo struct array by data, A and b
%   data: D x N, data in world frame
%   A: D x D x F, orientation matrices of each frame
%   b: D x F, position vectors of each frame
%   tmpTime: boolean, true for adding temporal term to data (default:false)
%   Demo: TPDemo struct, the demo
%   @TPGMMOne

if nargin < 5
    tmpTime = false;
end

D = size(data,1);
N = size(data,2);
F = size(A,3);
Demo = [];

if tmpTime
    % Adding temporal term to data in the first row
    D = D + 1;
    data = [linspace(0,1,N);data];
    Demo.A = repmat(eye(D),[1,1,F]);
    Demo.b = zeros(D,F);
    Demo.data = data;
    for f = 1:F
        Demo.A(2:D,2:D,f) = A(:,:,f);
        Demo.b(2:D,f) = b(:,f);
        for n = 1:N
%             Demo.TPData(:,f,n) = (Demo.A(:,:,f)) * data(:,n) + (Demo.b(:,f));
            Demo.TPData(:,f,n) = (Demo.A(:,:,f))' * (data(:,n) - (Demo.b(:,f)));
        end
    end
else
    % Original data
    Demo.A = A;
    Demo.b = b;
    Demo.data = data;
    Demo.TPData = zeros(D,F,N);
    for f = 1:F
        for n = 1:N
%             Demo.TPData(:,f,n) = A(:,:,f)*data(:,n) + b(:,f);
            Demo.TPData(:,f,n) = A(:,:,f)' * (data(:,n) - b(:,f));
        end
    end
end

end

