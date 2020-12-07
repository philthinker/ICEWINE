function [Data] = dataRegulate(obj,rawData)
%dataRegulate Regulate the raw optitrack body data.
%   rawData: N x 7, [ qx qy qz qw x y z] raw data
%   -------------------------------------------------
%   Data: N x 7, [ qw, qx, qy, qz, x, y, z], qw > 0, regulated data
%   @OptitrackDataOne

% Reorder
tmpQw = rawData(:,4);

Data = rawData;
Data(:,1) = tmpQw;
Data(:,2:4) = rawData(:,1:3);

% Unit quat. regulate
% Ambiguity elimination
tmpLID = tmpQw < 0;
Data(tmpLID,1:4) = -Data(tmpLID,1:4);
% Normalization
Data(:,1:4) = Data(:,1:4)./sqrt(diag(Data(:,1:4)*Data(:,1:4)'));

end

