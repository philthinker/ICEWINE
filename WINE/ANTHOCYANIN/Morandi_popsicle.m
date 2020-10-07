function [rgb] = Morandi_popsicle(index)
%Morandi_popsicle Morandi palette: Popsicle
%   index: integer, the color index
%   -------------------------------------------------
%   rgb: 1 x 3, rgb color value

popsicle_255 = ...
    [   133,  203,  205;
        251,  199,  141;
        167,  214,  118;
        168,  222,  224;
        249,  226,  174;
        209,  228,  182;
        248,  177,  216;
        229,  187,  239;
        237,  208,  192;    ];
    
popsicle_1 = popsicle_255/255;

% index = max([1,round(index)]);
index = mod(index,9);
if index == 0
    index = 9;
end

rgb = popsicle_1(index,:);

end

