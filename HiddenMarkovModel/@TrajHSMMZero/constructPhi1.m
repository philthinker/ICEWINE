function [Phi1,Phi0] = constructPhi1(obj,N,dt)
%constructPhi1 Construct the Phi matrix for reproduction
%   N: integer, num. of data required
%   dt: scalar, time difference
%   -------------------------------------------------
%   Phi1: N*DD*DP x N*DP, Phi matrix for DP-D data
%   Phi0: N*DD x N, Phi matrix for 1-D data
%   @TrajHSMMZero

DD = obj.DD;
DP = obj.D/obj.DD;

phi = zeros(DD);    % Basic transformation for 1 1-D data
phi(1,end) = 1;     % x(T) = x(T), dx(T) = (x(T) - x(T-1))/dt
for i = 2:DD
    % Pascal's Triangle
    phi(i,:) = (phi(i-1,:) - circshift(phi(i-1,:),[0,-1])) / dt;
end

%   Phi1: N*DD*DP x N*DP, Phi matrix for DP-D data
%   Phi0: N*DD x N, Phi matrix for 1-D data

% Transformation for N 1-D data
Phi0 = zeros(N*DD,N+DD-1); % Note that there are DD-1 more col.
for i = 1:N
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
Phi1 = kron(Phi0,eye(DP));    % Transformation for N DP-D data

end

