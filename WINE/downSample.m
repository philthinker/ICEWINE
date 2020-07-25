function [DataOut,Nout] = downSample(DataIn,downScalar)
%downSample Down sample the data to save time
%   DataIn: N x D, data input
%   downScalar: integer < N/2, the size scalar
%   DataOut: Nout x D, data output
%   Nout: integer < N, number of output data

S = round(downScalar);
N = size(DataIn,1);
if S >= N/2
    Nout = 2;
    DataOut = [DataIn(1,:);DataIn(end,:)];
    return
elseif S <= 1
    DataOut = DataIn;
    Nout = size(DataOut,1);
    return
end

indices = mod((1:N)',S) == 1;
DataOut = DataIn(indices,:);
Nout = size(DataOut,1);

end

