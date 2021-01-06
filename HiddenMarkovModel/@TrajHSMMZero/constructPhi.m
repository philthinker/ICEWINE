function [Phi,Phi1,Phi0] = constructPhi(obj,N,M,dt)
%constructPhi Construct the large sparse matrix Phi
%   nData: N, num. of data in one trajectory
%   nSample: M, num. of demos
%   -------------------------------------------------
%   Phi: N * nDeriv * D * M x N * D * M
%   Phi1: N * nDeriv * D x N * D
%   Phi0: N * nDeriv x N
%   @TrajHSMMZero

% 
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 


op1D = zeros(obj.DD);
op1D(1,end) = 1;
for i=2:obj.DD
	op1D(i,:) = (op1D(i-1,:) - circshift(op1D(i-1,:),[0,-1])) / dt;
end
op = zeros((obj.DD)*N, N);
op((obj.DD-1)*obj.DD+1:obj.DD*obj.DD, 1:obj.DD) = op1D;
Phi0 = zeros(N*obj.DD, N);
for t=0:N-obj.DD
	Phi0 = Phi0 + circshift(op, [obj.DD*t,t]);
end
%Handling of borders
for i=1:obj.DD-1
	op(obj.DD*obj.DD+1-i,:)=0; op(:,i)=0;
	Phi0 = Phi0 + circshift(op, [-i*obj.DD,-i]);
end
%Application to multiple dimensions and multiple demonstrations
Phi1 = kron(Phi0, eye(obj.nVarPos));
Phi = kron(eye(M), Phi1);

end

