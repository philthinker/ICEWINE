function [rgb] = Morandi_hydrangea(index)
%Morandi_hydrangea Morandi palette: Hydrangea
%   index: integer, the color index
%   -------------------------------------------------
%   rgb: 1 x 3, rgb color value

hydrangea_255 = ...
    [   250,  147,  151;    % r
        53,  100,  146;      % b
        255,  208,  166;    % y
        149,  196,  190;    % g
        248,  177,  216;    % r
        142,  211,  232;    % b
        209,  228,  182;    % g
        229,  187,  239;    % p
        162,  192,  230;    ];  % b
    
hydrangea_1 = hydrangea_255/255;

% index = max([1,round(index)]);
index = mod(index,9);
if index == 0
    index = 9;
end

rgb = hydrangea_1(index,:);

end

