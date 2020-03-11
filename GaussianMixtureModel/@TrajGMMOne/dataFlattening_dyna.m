function [dynaData,Ns,Phi] = dataFlattening_dyna(obj,Demos,dt)
%dataFlattening_dyna Flatten the data in demos and compute their diff.
%   If obj.tpFlag == false:
%   Demos: 1 x M struct array:
%   |   data: DPos x N, demo data (pos. only)
%   dt: scalar, time difference
%   dynaData: DPos*DD x N*M, demo data, individual Ns are also supported
%   Ns: 1 x M, num. of data in each demo
%   Phi: N*DD*DPos*M x N*DPos*M, Phi matrix for DPos-D data in M demos
%   @TrajGMMOne

if obj.tpFlag
    % TP-Traj-GMM
    
else
    % Ordinary Traj-GMM
    % Retrieve the raw position data
    [posData,Ns] = obj.dataRegulate(Demos); % [x(1),x(2),x(3),x(4),...]
    % Compute Phi
    Phi = obj.constructPhi(Demos,dt);
    % Re-arrange data in vector form
    DPos = (obj.nVar)/(obj.nDiff); % DPos = D/DD
    NM = size(posData,2);
    % Scale the data to avoid numerical computation problem (param_x_amplifer)
    x = reshape(posData, DPos*NM, 1)*obj.param_x_amplifier; % [x1(1),x2(1),x1(2),x2(2),x1(3),x2(3),...]'
    zeta = Phi * x;   % [x1(1), x2(1), x1d(1), x2d(1), x1(2), x2(2), x1d(2), x2d(2), ...]'
    % Include derivatives in dynaData
    dynaData = reshape(zeta, obj.nVar, NM);
end

end

