function demo_Riemannian_cov_GMR03
% GMR with vector as input and covariance data as output by relying on Riemannian manifold
%
% Writing code takes time. Polishing it and making it available to others takes longer! 
% If some parts of the code were useful for your research of for a better understanding 
% of the algorithms, please cite the related publications.
%
% @article{Jaquier17IROS,
%   author="Jaquier, N. and Calinon, S.",
%   title="Gaussian Mixture Regression on Symmetric Positive Definite Matrices Manifolds: 
%	    Application to Wrist Motion Estimation with s{EMG}",
%   year="2017",
%	  booktitle = "{IEEE/RSJ} Intl. Conf. on Intelligent Robots and Systems ({IROS})",
%	  address = "Vancouver, Canada"
% }
% 
% Copyright (c) 2017 Idiap Research Institute, http://idiap.ch/
% Written by Noémie Jaquier and Sylvain Calinon
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 
% PbDlib is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License version 3 as
% published by the Free Software Foundation.
% 
% PbDlib is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with PbDlib. If not, see <http://www.gnu.org/licenses/>.

addpath('./m_fcts/');

%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbData = 30; % Number of datapoints
nbSamples = 1; % Number of demonstrations
nbIter = 5; % Number of iteration for the Gauss Newton algorithm
nbIterEM = 5; % Number of iteration for the EM algorithm

model.nbStates = 5; % Number of states in the GMM
model.nbVar = 4; % Dimension of the manifold and tangent space (2D input + 2^2 covariance output)
model.nbVarCovOut = model.nbVar + model.nbVar*(model.nbVar-1)/2; % Dimension of the output covariance
model.dt = 1E-1; % Time step duration
model.params_diagRegFact = 1E-2; % Regularization of covariance

% Initialisation of the covariance transformation for input-output covariance
[covOrder4to2, covOrder2to4] = set_covOrder4to2(model.nbVar);
% Initialisation of the covariance transformation for output covariance
[covOrder4to2_out, ~] = set_covOrder4to2(model.nbVar-2);

% Tensor regularization term
tensor_diagRegFact_mat = eye(model.nbVar + model.nbVar * (model.nbVar-1)/2);
tensor_diagRegFact = covOrder2to4(tensor_diagRegFact_mat);


%% Generate covariance data from rotating covariance
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time(1,:) = [1:nbData] * model.dt;

Vdata = eye(model.nbVar-2);
Ddata = eye(model.nbVar-2);

X = zeros(model.nbVar,model.nbVar,nbData*nbSamples);
% Input as first eigenvector, output as covariance matrix
for t = 1:nbData
	Ddata(1,1) = t * 1E-1;
	a = pi/2 * t * 1E-1;
	R = [cos(a) -sin(a); sin(a) cos(a)];
	V2 = R * Vdata;
	X(1:2,1:2,t) = diag(V2(:,end));
	X(3:4,3:4,t) = V2 * Ddata * V2';
end
xIn = X(1:2,1:2,:);


%% GMM parameters estimation 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Learning...');
in=1:2; out=3:model.nbVar;
id = cov_findNonZeroID(in, out, 1, 0);
model = spd_init_GMM_kbins(X, model, nbSamples,out);
model.Mu = zeros(size(model.MuMan));

L = zeros(model.nbStates, nbData*nbSamples);
Xts = zeros(model.nbVar, model.nbVar, nbData*nbSamples, model.nbStates);
for nb=1:nbIterEM
	
	% E-step
	for i=1:model.nbStates
		Xts(in,in,:,i) = X(in,in,:)-repmat(model.MuMan(in,in,i),1,1,nbData*nbSamples);
		Xts(out,out,:,i) = logmap(X(out,out,:), model.MuMan(out,out,i));
		
		% Compute probabilities using the reduced form (computationally
		% less expensive than complete form)
		xts = symMat2Vec(Xts(:,:,:,i));
		MuVec = symMat2Vec(model.Mu(:,:,i));
		SigmaVec = covOrder4to2(model.Sigma(:,:,:,:,i));

% 		L(i,:) = model.Priors(i) * gaussPDF2(xts, MuVec, SigmaVec);
		L(i,:) = model.Priors(i) * gaussPDF(xts(id,:), MuVec(id,:), SigmaVec(id,id));

	end
	GAMMA = L ./ repmat(sum(L,1)+realmin, model.nbStates, 1);
	H = GAMMA ./ repmat(sum(GAMMA,2)+realmin, 1, nbData*nbSamples);
	% M-step
	for i=1:model.nbStates
		% Update Priors
		model.Priors(i) = sum(GAMMA(i,:)) / (nbData*nbSamples);
		
		% Update MuMan
		for n=1:nbIter
			uTmpTot = zeros(model.nbVar,model.nbVar);
			uTmp = zeros(model.nbVar,model.nbVar,nbData*nbSamples);
			uTmp(in,in,:) = X(in,in,:)-repmat(model.MuMan(in,in,i),1,1,nbData*nbSamples);
			uTmp(out,out,:) = logmap(X(out,out,:), model.MuMan(out,out,i));
			
			for k = 1:nbData*nbSamples
				uTmpTot = uTmpTot + uTmp(:,:,k) .* H(i,k);
			end
			model.MuMan(in,in,i) = uTmpTot(in,in) + model.MuMan(in,in,i);
			model.MuMan(out,out,i) = expmap(uTmpTot(out,out), model.MuMan(out,out,i));
		end
		
		% Update Sigma
		model.Sigma(:,:,:,:,i) = zeros(model.nbVar,model.nbVar,model.nbVar,model.nbVar);
		for k = 1:nbData*nbSamples
			model.Sigma(:,:,:,:,i) = model.Sigma(:,:,:,:,i) + H(i,k) .* outerprod(uTmp(:,:,k),uTmp(:,:,k));
		end
		model.Sigma(:,:,:,:,i) = model.Sigma(:,:,:,:,i) + tensor_diagRegFact.*model.params_diagRegFact;
	end
end

% Eigendecomposition of Sigma
for i=1:model.nbStates
	[~, V(:,:,:,i), D(:,:,i)] = covOrder4to2(model.Sigma(:,:,:,:,i));
end


%% GMR (version with single optimization loop)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Regression...');

in=1:2; out=3:model.nbVar;
nbVarOut = length(out);

uhat = zeros(nbVarOut,nbVarOut,nbData);
xhat = zeros(nbVarOut,nbVarOut,nbData);
uOut = zeros(nbVarOut,nbVarOut,model.nbStates,nbData);
SigmaTmp = zeros(model.nbVarCovOut,model.nbVarCovOut,model.nbStates);
expSigma = zeros(nbVarOut,nbVarOut,nbVarOut,nbVarOut,nbData);
H = [];
for t=1:nbData
	% Compute activation weight
	for i=1:model.nbStates
		% Compute H using the reduced form
		xInVec = symMat2Vec(xIn(:,:,t));
		MuManInVec = symMat2Vec(model.MuMan(in,in,i));
		MuInVec = symMat2Vec(model.Mu(in,in,i));
		SigmaInVec = covOrder4to2_out(model.Sigma(in,in,in,in,i));
		H(i,t) = model.Priors(i) * gaussPDF(xInVec-MuManInVec, MuInVec, SigmaInVec);
	end
	H(:,t) = H(:,t) / sum(H(:,t)+realmin);
	
	% Compute conditional mean (with covariance transportation)
	if t==1
		[~,id] = max(H(:,t));
		xhat(:,:,t) = model.MuMan(out,out,id); %Initial point
	else
		xhat(:,:,t) = xhat(:,:,t-1);
	end
	for n=1:nbIter
		uhat(:,:,t) = zeros(nbVarOut,nbVarOut);
		for i=1:model.nbStates
			% Transportation of covariance from model.MuMan(outMan,i) to xhat(:,t)
			S1 = model.MuMan(out,out,i);
			S2 = xhat(:,:,t);
			Ac = blkdiag(eye(length(in)),transp(S1,S2));
			
			% Parallel transport of eigenvectors
			for j = 1:size(V,3)
				pV(:,:,j,i) = Ac * D(j,j,i)^.5 * V(:,:,j,i) * Ac';
			end
			
			% Parallel transported sigma (reconstruction from eigenvectors)
			pSigma(:,:,:,:,i) = zeros(size(model.Sigma(:,:,:,:,i)));
			for j = 1:size(V,3)
				pSigma(:,:,:,:,i) = pSigma(:,:,:,:,i) + outerprod(pV(:,:,j,i),pV(:,:,j,i));
			end
			
			[SigmaTmp(:,:,i), pV(:,:,:,i), pD(:,:,i)] = covOrder4to2(pSigma(:,:,:,:,i));
			
			% Gaussian conditioning on the tangent space
			uOut(:,:,i,t) = logmap(model.MuMan(out,out,i), xhat(:,:,t)) + ...
				tensor4o_mult(tensor4o_div(pSigma(out,out,in,in,i),pSigma(in,in,in,in,i),covOrder4to2_out), ...
				(xIn(:,:,t)-model.MuMan(in,in,i)));
			
			uhat(:,:,t) = uhat(:,:,t) + uOut(:,:,i,t) * H(i,t);
		end
		
		xhat(:,:,t) = expmap(uhat(:,:,t), xhat(:,:,t));
	end
	
	% Compute conditional covariances (note that since uhat=0, the final part in the GMR computation is dropped)
	for i=1:model.nbStates
		SigmaOutTmp = pSigma(out,out,out,out,i) ...
			- tensor4o_mult(tensor4o_div(pSigma(out,out,in,in,i),pSigma(in,in,in,in,i),covOrder4to2_out),pSigma(in,in,out,out,i));
		expSigma(:,:,:,:,t) = expSigma(:,:,:,:,t) + H(i,t) * (SigmaOutTmp + outerprod(uOut(:,:,i,t),uOut(:,:,i,t)));
	end
end

model.MuMan = real(model.MuMan);


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('PaperPosition',[0 0 8 8],'position',[10,10,1350,650]);
clrmap = lines(model.nbStates);

subplot(2,1,1); hold on; axis off;
sc = 1E1;
for t=1:size(X,3)
	plotGMM(diag(X(in,in,t))*sc, X(out,out,t), [.6 .6 .6], .1);
end
for i=1:model.nbStates
	plotGMM(diag(model.MuMan(in,in,i))*sc, model.MuMan(out,out,i), clrmap(i,:), .3);
end
for t=1:nbData
	plotGMM(diag(X(in,in,t))*sc, xhat(:,:,t), [0 1 0], .1);
end

subplot(2,1,2); hold on;
for i=1:model.nbStates
	plot(time, H(i,:),'linewidth',2,'color',clrmap(i,:));
end
axis([time(1), time(end), 0, 1.02]);
set(gca,'xtick',[],'ytick',[]);
xlabel('t'); ylabel('h_i');

% pause;
% close all;
end


%% Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function X = expmap(U,S)
% Exponential map (SPD manifold)
N = size(U,3);
for n = 1:N
	X(:,:,n) = S^.5 * expm(S^-.5 * U(:,:,n) * S^-.5) * S^.5;
end
end

function U = logmap(X,S)
% Logarithm map (SPD manifold)
N = size(X,3);
for n = 1:N
	U(:,:,n) = S^.5 * logm(S^-.5 * X(:,:,n) * S^-.5) * S^.5;
end
end

function Ac = transp(S1,S2)
% Parallel transport (SPD manifold)
t = 1;
U = logmap(S2,S1);
Ac = S1^.5 * expm(0.5 .* t .* S1^-.5 * U * S1^-.5) * S1^-.5;
%Computationally economic way: Ac = (S2/S1)^.5
end

function M = spdMean(setS, nbIt)
% Mean of SPD matrices on the manifold
if nargin == 1
	nbIt = 10;
end
M = setS(:,:,1);

for i=1:nbIt
	L = zeros(size(setS,1),size(setS,2));
	for n = 1:size(setS,3)
		L = L + logm(M^-.5 * setS(:,:,n)* M^-.5);
	end
	M = M^.5 * expm(L./size(setS,3)) * M^.5;
end

end

function model = spd_init_GMM_kbins(Data, model, nbSamples, spdDataId)
% K-Bins initialisation by relying on SPD manifold Parameters
nbData = size(Data,3) / nbSamples;
if ~isfield(model,'params_diagRegFact')
	model.params_diagRegFact = 1E-4; %Optional regularization term to avoid numerical instability
end
e0tensor = zeros(model.nbVar, model.nbVar, model.nbVar, model.nbVar);
for i = 1:model.nbVar
	e0tensor(i,i,i,i) = 1;
end
% Delimit the cluster bins for the first demonstration
tSep = round(linspace(0, nbData, model.nbStates+1));

% Compute statistics for each bin
for i=1:model.nbStates
	id=[];
	for n=1:nbSamples
		id = [id (n-1)*nbData+[tSep(i)+1:tSep(i+1)]];
	end
	model.Priors(i) = length(id);
	
	% Mean computed on SPD manifold for parts of the data belonging to the
	% manifold
	if nargin < 4
		model.MuMan(:,:,i) = spdMean(Data(:,:,id));
	else
		model.MuMan(:,:,i) = mean(Data(:,:,id),3);
		if iscell(spdDataId)
			for c = 1:length(spdDataId)
				model.MuMan(spdDataId{c},spdDataId{c},i) = spdMean(Data(spdDataId{c},spdDataId{c},id),3);
			end
		else
			model.MuMan(spdDataId,spdDataId,i) = spdMean(Data(spdDataId,spdDataId,id),3);
		end
	end
	
	% Parts of data belonging to SPD manifold projected to tangent space at
	% the mean to compute the covariance tensor in the tangent space
	DataTgt = zeros(size(Data(:,:,id)));
	if nargin < 4
		DataTgt = logmap(Data(:,:,id),model.MuMan(:,:,i));
	else
		DataTgt = Data(:,:,id);
		if iscell(spdDataId)
			for c = 1:length(spdDataId)
				DataTgt(spdDataId{c},spdDataId{c},:) = logmap(Data(spdDataId{c},spdDataId{c},id),model.MuMan(spdDataId{c},spdDataId{c},i));
			end
		else
			DataTgt(spdDataId,spdDataId,:) = logmap(Data(spdDataId,spdDataId,id),model.MuMan(spdDataId,spdDataId,i));
		end
	end

	model.Sigma(:,:,:,:,i) = computeCov(DataTgt) + e0tensor.*model.params_diagRegFact;
	
end
model.Priors = model.Priors / sum(model.Priors);
end

function M = tensor2mat(T, rows, cols)
% Matricisation of a tensor
% The rows, respectively columns of the matrix are 'rows', respectively
% 'cols' of the tensor.
if nargin <=2
	cols = [];
end

sizeT = size(T);
N = ndims(T);

if isempty(rows)
	rows = 1:N;
	rows(cols) = [];
end
if isempty(cols)
	cols = 1:N;
	cols(rows) = [];
end

if any(rows(:)' ~= 1:length(rows)) || any(cols(:)' ~= length(rows)+(1:length(cols)))
	T = permute(T,[rows(:)' cols(:)']);
end

M = reshape(T,prod(sizeT(rows)), prod(sizeT(cols)));

end

function T = tensor4o_mult(A,B)
% Multiplication of two 4th-order tensors A and B
if ndims(A) == 4 || ndims(B) == 4
	sizeA = size(A);
	sizeB = size(B);
	if ismatrix(A) 
		sizeA(3:4) = [1,1];
	end
	if ismatrix(B)
		sizeB(3:4) = [1,1];
	end

	if sizeA(3) ~= sizeB(1) || sizeA(4) ~= sizeB(2)
		error('Dimensions mismatch: two last dim of A should be the same than two first dim of B.');
	end

% multiplication with outer product, more expensive (loops)	
% 	T = zeros(sizeA(1),sizeA(2),sizeB(3),sizeB(4));
% 
% 	for i = 1:sizeA(3)
% 		for j = 1:sizeA(4)
% 			T = T + outerprod(A(:,:,i,j),permute(B(i,j,:,:),[3,4,1,2]));
% 		end
% 	end
	T = reshape(tensor2mat(A,[1,2]) * tensor2mat(B,[1,2]), [sizeA(1),sizeA(2),sizeB(3),sizeB(4)]);
	
else
	if ismatrix(A) && isscalar(B)
		T = A*B;
	else
		error('Dimensions mismatch.');
	end
end
end

function T = tensor4o_div(A,B,covOrder4to2)
% Division of two 4th-order tensors A and B
if ndims(A) == 4 || ndims(B) == 4
	sizeA = size(A);
	sizeB = size(B);
	if ismatrix(A)
		sizeA(3:4) = [1,1];
	end
	if ismatrix(B)
		T = A/B;
	else
		if sizeA(3) ~= sizeB(1) || sizeA(4) ~= sizeB(2)
			error('Dimensions mismatch: two last dim of A should be the same than two first dim of B.');
		end

		[~, V, D] = covOrder4to2(B);
		invB = zeros(size(B));
		for j = 1:size(V,3)
			invB = invB + D(j,j)^-1 .* outerprod(V(:,:,j),V(:,:,j));
		end
		
% multiplication with outer product, more expensive (loops)		
% 		T = zeros(sizeA(1),sizeA(2),sizeB(3),sizeB(4));
% 
% 		for i = 1:sizeA(3)
% 			for j = 1:sizeA(4)
% 				T = T + outerprod(A(:,:,i,j),permute(invB(i,j,:,:),[3,4,1,2]));
% 			end
% 		end

		% multiplication using matricisation of tensors
		T = reshape(tensor2mat(A,[1,2]) * tensor2mat(invB,[1,2]), [sizeA(1),sizeA(2),sizeB(3),sizeB(4)]);
	end
else
	if ismatrix(A) && isscalar(B)
		T = A/B;
	else
		error('Dimensions mismatch.');
	end
end
end

function prob = gaussPDF2(Data, Mu, Sigma)
% Likelihood of datapoint(s) to be generated by a Gaussian parameterized by
% center and covariance. The inverse and determinant of the covariance are
% computed using the eigenvalue decomposition.
[nbVar,nbData] = size(Data);
Data = Data' - repmat(Mu',nbData,1);
[V,D] = eig(Sigma);
SigmaInv = V*diag(diag(D).^-1)*V';
prob = sum((Data*SigmaInv).*Data, 2);
prob = exp(-0.5*prob) / sqrt((2*pi)^nbVar * abs(det(Sigma)) + realmin);
end

function S = computeCov(Data)
% Compute the 4th-order covariance of matrix data
d = size(Data,1);
N = size(Data,3);

Data = Data-repmat(mean(Data,3),1,1,N);

S = zeros(d,d,d,d);
for i = 1:N
	S = S + outerprod(Data(:,:,i),Data(:,:,i));
end
S = S./(N-1);
end

function v = symMat2Vec(S)
% Reduced vectorisation of a symmetric matrix
[d,~,N] = size(S);

v = zeros(d+d*(d-1)/2,N);
for n = 1:N
	v(1:d,n) = diag(S(:,:,n));
	
	row = d+1;
	for i = 1:d-1
		v(row:row + d-1-i,n) = sqrt(2).*S(i+1:end,i,n);
		row = row + d-i;
	end
end
end

function id = cov_findNonZeroID(in, out, isVec_in, isVec_out)
% Return locations of non-zero elements in a block diagonal matrix where 
% block element may be matrices or vectors
	if nargin == 2
		isVec_in = 0;
		isVec_out = 0;
	end
	
	numberMat = zeros(out(end));
	if ~isVec_in
		numberMat(in,in) = ones(length(in));
	else
		numberMat(in,in) = eye(length(in));
	end
	if ~isVec_out
		numberMat(out,out) = ones(length(out));
	else
		numberMat(out,out) = eye(length(out));
	end
	
	numberVec = logical(symMat2Vec(numberMat))';
	numberVec = numberVec.*[1:length(numberVec)];
	
	id = nonzeros(numberVec)';
	
end

function [covOrder4to2, covOrder2to4] = set_covOrder4to2(dim)
% Set the factors necessary to simplify a 4th-order covariance of symmetric
% matrix to a 2nd-order covariance. The dimension ofthe 4th-order covariance is 
% dim x dim x dim x dim. Return the functions covOrder4to2 and covOrder2to4. 
% This function must be called one time for each covariance's size.
newDim = dim+dim*(dim-1)/2;

% Computation of the indices and coefficients to transform 4th-order
% covariances to 2nd-order covariances
sizeS = [dim,dim,dim,dim];
sizeSred = [newDim,newDim];
id = [];
idred = [];
coeffs = [];

% left-up part
for k = 1:dim
	for m = 1:dim
		id = [id,sub2ind(sizeS,k,k,m,m)];
		idred = [idred,sub2ind(sizeSred,k,m)];
		coeffs = [coeffs,1];
	end
end

% right-down part
row = dim+1; col = dim+1;
for k = 1:dim-1
	for m = k+1:dim
		for p = 1:dim-1
			for q = p+1:dim
				id = [id,sub2ind(sizeS,k,m,p,q)];
				idred = [idred,sub2ind(sizeSred,row,col)];
				coeffs = [coeffs,2];
				col = col+1;
			end
		end
		row = row + 1;
		col = dim+1;
	end
end

% side-parts
for k = 1:dim
	col = dim+1;
	for p = 1:dim-1
		for q = p+1:dim
			id = [id,sub2ind(sizeS,k,k,p,q)];
			idred = [idred,sub2ind(sizeSred,k,col)];
			id = [id,sub2ind(sizeS,k,k,p,q)];
			idred = [idred,sub2ind(sizeSred,col,k)];
			coeffs = [coeffs,sqrt(2),sqrt(2)];
			col = col+1;
		end
	end
end

% Computation of the indices and coefficients to transform eigenvectors to
% eigentensors
sizeV = [dim,dim,newDim];
sizeVred = [newDim,newDim];
idEig = [];
idredEig = [];
coeffsEig = [];

for n = 1:newDim
	% diagonal part
	for j = 1:dim
		idEig = [idEig,sub2ind(sizeV,j,j,n)];
		idredEig = [idredEig,sub2ind(sizeVred,j,n)];
		coeffsEig = [coeffsEig,1];
	end
	
	% side parts
	j = dim+1;
	for k = 1:dim-1
		for m = k+1:dim
			idEig = [idEig,sub2ind(sizeV,k,m,n)];
			idredEig = [idredEig,sub2ind(sizeVred,j,n)];
			idEig = [idEig,sub2ind(sizeV,m,k,n)];
			idredEig = [idredEig,sub2ind(sizeVred,j,n)];
			coeffsEig = [coeffsEig,1/sqrt(2),1/sqrt(2)];
			j = j+1;
		end
	end
end

function [Sred, V, D] = def_covOrder4to2(S)
	Sred = zeros(newDim,newDim);
	Sred(idred) = bsxfun(@times,S(id),coeffs);
	[v,D] = eig(Sred);
	V = zeros(dim,dim,newDim);
	V(idEig) = bsxfun(@times,v(idredEig),coeffsEig);
end
function [S, V, D] = def_covOrder2to4(Sred) 
	[v,D] = eig(Sred);
	V = zeros(dim,dim,newDim);
	V(idEig) = bsxfun(@times,v(idredEig),coeffsEig);

	S = zeros(dim,dim,dim,dim);
	for i = 1:size(V,3)
		S = S + D(i,i).*outerprod(V(:,:,i),V(:,:,i));
	end
end

covOrder4to2 = @def_covOrder4to2;
covOrder2to4 = @def_covOrder2to4;

end