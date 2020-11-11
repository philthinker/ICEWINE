function [dataOut,NOut] = curtialTail(dataIn,mode)
%curtialTial Curtial the trivial "tail" of dataIn
%   dataIn: N x D or D x N, input data
%   mode: 0 for N x D dataIn, 1 for D x N dataIn (default:0)
%   dataOut: NOut x D or D x NOut, output data
%   NOut: integer, number of data in dataOut

if nargin < 2
    mode = 0;
end

if mode == 1
    % D x N dataIn
    N = size(dataIn,2);
    for i = 0:N-2
        if any(dataIn(:,N-i) ~= dataIn(:,N-i-1))
            break
        end
    end
    dataOut = dataIn(:,1:N-i-1);
    NOut = N-i-1;
else
    % N x D dataIn
    N = size(dataIn,1);
    for i = 0:N-2
        if any(dataIn(N-i,:) ~= dataIn(N-i-1,:))
            break
        end
    end
    dataOut = dataIn(1:N-i-1,:);
    NOut = N-i-1;
end

end

