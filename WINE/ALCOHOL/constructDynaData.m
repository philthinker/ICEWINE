function [dynaData] = constructDynaData(DataP, DD, dt)
%constructDynaData Construct the dynamic data given the position data
%   [p', v', a', j', ....]'. We always assume zero initial condition.
%   DataP: DP x N, position data
%   DD: Integer > 1, order of derivation
%   dt: Scalar, time difference
%   -------------------------------------------------
%   Data: DP*DD x N, dynamic data

DD = max([ round(DD), 1]);
dt = max(1e-6, dt);
DP = size(DataP,1);
N = size(DataP,2);
dynaData = zeros(DD * DP, N);
dynaData(1:DP, :) = DataP;

if N < DD
    return;
end
if DD < 1
    return;
end

for i = 1:DD-1
    dynaData(i*DP+1 : (i+1)*DP, i+1:end) = ( dynaData((i-1)*DP+1: i*DP, 1+i:end) - dynaData((i-1)*DP+1: i*DP, i:end-1) )/dt;
end

end

