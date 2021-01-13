function [CellDataOut] = extractDataFieldAsCell(Data,field)
%extractDataFieldAsCell Extract the field data in struct array as cell
%   Data: 1 x M or M x 1, struct array with field 'field'.
%   field: String, the name of required field
%   -------------------------------------------------
%   CellDataOut: 1 x M or M x 1, cell

M = length(Data);
CellDataOut = cell(size(Data));

if isfield(Data(1),field)
    for i = 1:M
        CellDataOut{i} = Data(i).(field);
    end
end

end