function [DataOut] = slideAverageWindow(DataIn,wid)
%slideAverageWindow Slide window of average operator. We always assume the
%window is sliding along the dim. 2 (row).
%   DataIn: D x N, data input
%   wid: Integer, window width
%   -------------------------------------------------
%   DataOut: D x N, data output

DataOut = DataIn;
N = size(DataIn, 2);
wid = max([1, round(wid)]);

if N < 2 || N <= wid
    return;
end

% The 1st and last data is kept.
for i = 2:N-1
    if i <= wid/2
        tmpData = DataIn(:,1:i+floor(wid/2));
    elseif N <= i+wid/2
        tmpData = DataIn(:,i-floor(wid/2):N);
    else
        tmpData = DataIn(:,i-floor(wid/2):i+floor(wid/2));
    end
    DataOut(:,i) = mean(tmpData,2);
end

end

