function [Phi,Phi1,Phi0] = constructPhi(obj,nData,nSample)
%constructPhi Construct the large sparse matrix Phi
%   nSample: M, num. of demos (optional)
%   nData: N, num. of data in one trajectory (optional)
%   Phi: N * nDeriv * D * M x N * D * M
%   Phi1: N * nDeriv * D x N * D
%   Phi0: N * nDeriv x N
%   @TrajGMM

% 
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 

if nargin < 3
    nSample = obj.nSample;
    if nargin < 2
        nData = obj.nData;
    end
end

op1D = zeros(obj.nDeriv);
op1D(1,end) = 1;
for i=2:obj.nDeriv
	op1D(i,:) = (op1D(i-1,:) - circshift(op1D(i-1,:),[0,-1])) / obj.dt;
end
op = zeros((obj.nDeriv)*nData, nData);
op((obj.nDeriv-1)*obj.nDeriv+1:obj.nDeriv*obj.nDeriv, 1:obj.nDeriv) = op1D;
Phi0 = zeros(nData*obj.nDeriv, nData);
for t=0:nData-obj.nDeriv
	Phi0 = Phi0 + circshift(op, [obj.nDeriv*t,t]);
end
%Handling of borders
for i=1:obj.nDeriv-1
	op(obj.nDeriv*obj.nDeriv+1-i,:)=0; op(:,i)=0;
	Phi0 = Phi0 + circshift(op, [-i*obj.nDeriv,-i]);
end
%Application to multiple dimensions and multiple demonstrations
Phi1 = kron(Phi0, eye(obj.nVarPos));
Phi = kron(eye(nSample), Phi1);

end

