function [Data,A,b] = extractTPCarte(dataCarte)
%extractTPCarte Extract the TP param. from Cartesian data
%   dataCarte: D+1 x D+1 x N, Cartesian data in the form of SE(3) or SE(2)
%   Data: D x N, position data
%   A: D x D x 2, rotation amtrics
%   b: D x 2, displacements

D = size(dataCarte,1) - 1;
Data = permute( dataCarte(1:D,D+1,:),[1,3,2] );
A = repmat(eye(D),[1,1,2]);
b = zeros(D,2);

A(:,:,1) = dataCarte(1:D,1:D,1);
A(:,:,2) = dataCarte(1:D,1:D,end);
b(:,1) = permute(dataCarte(1:D,D+1,1),[1,3,2]);
b(:,2) = permute(dataCarte(1:D,D+1,end),[1,3,2]);

end

