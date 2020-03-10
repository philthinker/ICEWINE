function [Phi] = constructPhi(obj,Demos,dt)
%constructPhi Construct the Phi matrix for demo data. We assume that:
%   dx(t) = (x(t) - x(t-1))/dt
%   ddx(t) = (dx(t) - dx(t-1))/dt = (x(t) - 2x(t-1) + x(t-2))/dt2
%   and the initial vel., acc. and jer. are all zero.
%   Demos: 1 x M struct array:
%   |   data: D x N, demo data
%   dt: scalar, time difference
%   Phi: N*DD*DPos*M x N*DPos*M, Phi matrix for DPos-D data in M demos
%   @TrajGMMOne

DD = obj.nDiff;
M = length(Demos);
DPos = size(Demos(1).data,1);

phi = zeros(DD);  % Basic transformation for 1 1-D data
phi(1,end) = 1; % x(T) = x(T)
for i = 2:DD
    % Pascal's Triangle
    phi(i,:) = (phi(i-1,:) - circshift(phi(i-1,:),[0,-1])) / dt;
end

%   Phi1: N*DD*DPos x N*DPos, Phi matrix for DPos-D data
%   Phi0: N*DD x N, Phi matrix for 1-D data
Phi = [];
for m = 1:M
    % For the m-th demo
    tmpN = size(Demos(m).data,2);
    % Transformation for N 1-D data
    Phi0 = zeros(tmpN*DD,tmpN+DD-1); % Note that there are DD-1 more col.
    for i = 1:tmpN
        Phi0((i-1)*DD+1:i*DD,i:i+DD-1) = phi;
    end
    Phi0 = Phi0(:,DD:end);  % There are DD-1 more col., get rid of them.
    % Handle the border of Phi0
    for i = 1:DD-1          % Block
        for j = i+1:DD      % Row
            for k = 1:i     % Column
                Phi0((i-1)*DD+j,k) = 0; % The initial vel., acc. and jer. are zero
            end
        end
    end
    Phi1 = kron(Phi0,eye(DPos));    % Transformation for N DPos-D data
    Phi = blkdiag(Phi,Phi1);        % Transformation for N DPos-D data in M demos
end

% Phi = kron(eye(M),Phi1);    

