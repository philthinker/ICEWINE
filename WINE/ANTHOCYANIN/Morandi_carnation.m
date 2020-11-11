function [rgb] = Morandi_carnation(index)
%Morandi_popsicle Morandi palette: Popsicle
%   index: integer, the color index
%   -------------------------------------------------
%   rgb: 1 x 3, rgb color value

carnation_255 = ...
    [   250, 147, 151;  % red
        54, 124, 175;  % blue
        255, 208, 166;  % orange
        149, 196, 190;  % green
        229, 187, 239;  % pink
        142, 211, 232;  % azure
        157, 135, 174;  % purple
        209, 228, 182;  % grass green
        251, 214, 49;    % yellow
        ];
    
carnation_1 = carnation_255/255;

% index = max([1,round(index)]);
index = mod(index,9);
if index == 0
    index = 9;
end

rgb = carnation_1(index,:);

end

