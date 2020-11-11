function [dataMatrix] = cell2matrix(dataCell,columnORrow)
%cell2matrix Transform a cell into its matrix form by combining those data
%   dataCell: 1 x M cells
%   columnORrow: integer, 0 for column data and 1 for row data (default: 0)
%   dataMatrix: MN x D (columnORrow == 1) or D x MN (otherwise)

if nargin < 2
    columnORrow = 0;
end

M = length(dataCell);
D = size(dataCell{1},2);
tmpN = zeros(1,M);
if columnORrow == 1
    % row data
    for i = 1:M
        tmpN(i) = size(dataCell{i},1);
    end
    dataMatrix = zeros(sum(tmpN), D);
    tmpIndex = 1;
    for i = 1:M
        dataMatrix(tmpIndex:tmpIndex+tmpN(i)-1, : ) = dataCell{i};
        tmpIndex = tmpIndex + tmpN(i);
    end
else
    % column data
    for i = 1:M
        tmpN(i) = size(dataCell{i},2);
    end
    dataMatrix = zeros(size(dataCell{1},1),sum(tmpN));
    tmpIndex = 1;
    for i = 1:M
        dataMatrix(:,tmpIndex:tmpIndex+tmpN(i)-1) = dataCell{i};
        tmpIndex = tmpIndex + tmpN(i);
    end
end

end

