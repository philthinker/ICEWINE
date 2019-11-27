%SO3Study

% Haopeng Hu
% 2019.10.31
% All rights reserved

N = 100;

SO3Data_rand = rpy2r(rand(N,3)*pi);
SO3Data_z = rpy2r([linspace(0,pi,N)',zeros(N,2)]);

V = zeros(3,3,N);
D = zeros(3,3,N);
for i = 1:N
    [V(:,:,i),D(:,:,i)] = eig(SO3Data_z(:,:,i));
end