function [demo,cutIndices] = demoAssemble(demoFor,demoLat,D)
%demoAssemble Assemble the D-dim demos data together
%   demoFor: 1 x M cell, former demos
%   demoLat:  1 x M cell, latter demos
%   D: integer, dimension of the data (row-major)
%   demo: 1 x M cell, assembled demos
%   cutIndices: 1 x M, the last indices of the former demos

D = round(D);
M = min([length(demoFor),length(demoLat)]);
demo = cell(1,M);
cutIndices = zeros(1,M);
for i = 1:M
    cutIndices(i) = size(demoFor{i},1);
    demo{i} = zeros(cutIndices(i)+size(demoLat{i},1),D);
    demo{i}(1:cutIndices(i),:) = demoFor{i}(:,1:D);
    demo{i}(cutIndices(i)+1:end,:) = demoLat{i}(:,1:D);
end

end

