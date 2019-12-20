function Data = dataThrow(obj,DataIn,W)
%dataThrow Only keep the first element each W elements
%   DataIn: D x N or D1 x D2 x N, data
%   W: integer, the window
%   @PandaSake

if size(DataIn,2) < W
    Data = DataIn;
else
    if size(DataIn,3) > 1
        % D1 x D2 x N
        Data = zeros(size(DataIn,1),size(DataIn,2),ceil(size(DataIn,3)/W));
        tmpIndex = 1;
        for i = 1:W:size(DataIn,3)
            Data(:,:,tmpIndex) = DataIn(:,:,i);
            tmpIndex = tmpIndex + 1;
        end
    else
        % D x N
        Data = zeros(size(DataIn,1), ceil(size(DataIn,2)/W));
        tmpIndex = 1;
        for i = 1:W:size(DataIn,2)
            Data(:,tmpIndex) = DataIn(:,i);
            tmpIndex = tmpIndex + 1;
        end
    end
end

end

