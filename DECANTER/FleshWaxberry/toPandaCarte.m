function [dataOut] = toPandaCarte(dataIn,dataPlus,mod)
%toPandaCarte Generate Cartesian data used for Franka Panda
%   dataIn: 3 x N, position data
%   dataPlus: ? x ?, auxiliary data
%   mod: integer, reserved
%   dataOut: 16 x N, Cartesian data

if nargin < 3
    mod = 0;
end

if mod == 1
    % dataPlus serves as orientation: 1 x 16
    N = size(dataIn,2);
    dataOut = repmat(dataPlus,[N,1]);
    dataOut(:,13:15) = dataIn';
else
%     N = size(dataIn,2);
    dataOut = dataIn';
end

end

