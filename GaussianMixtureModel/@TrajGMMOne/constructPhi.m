function [Phi,Phi1,Phi0] = constructPhi(obj,N,DD,DPos,M,dt)
%constructPhi Construct the Phi matrix
%   N: integer, N
%   DD: integer, nDiff,DD
%   DPos: integer, nPosVar,DPos
%   M: integer, num. of demos
%   dt: scalar, time difference
%   Phi: N*DD*DPos*M x N*DPos*M, Phi matrix forDPos-D data in M demos
%   Phi1: N*DD*DPos x N*DPos, Phi matrix for DPos-D data
%   Phi0: N*DD x N, Phi matrix for 1-D data
%   @TrajGMMOne



%{
op1D = zeros(obj.nDiff);
op1D(1,end) = 1;
for i=2:obj.nDiff
	op1D(i,:) = (op1D(i-1,:) - circshift(op1D(i-1,:),[0,-1])) / obj.dt;
end
op = zeros((obj.nDiff)*nData, nData);
op((obj.nDiff-1)*obj.nDiff+1:obj.nDiff*obj.nDiff, 1:obj.nDiff) = op1D;
Phi0 = zeros(nData*obj.nDiff, nData);
for t=0:nData-obj.nDiff
	Phi0 = Phi0 + circshift(op, [obj.nDiff*t,t]);
end
%Handling of borders
for i=1:obj.nDiff-1
	op(obj.nDiff*obj.nDiff+1-i,:)=0; op(:,i)=0;
	Phi0 = Phi0 + circshift(op, [-i*obj.nDiff,-i]);
end
%Application to multiple dimensions and multiple demonstrations
Phi1 = kron(Phi0, eye(obj.nVarPos));
Phi = kron(eye(nSample), Phi1);

end
%}
