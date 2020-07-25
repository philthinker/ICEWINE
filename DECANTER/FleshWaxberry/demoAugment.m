function [DemosOut,M, N] = demoAugment(Demos,field,N)
%demoAugment Augment the demos of the same size N
%   Demos: 1 x M struct array, demos
%   field: 1 x Nf cell of strings, fields to be augemented
%   DemosOut: struct with fields given by 'field' arugment

M = length(Demos);
Nf = length(field);
tmpD = zeros(Nf,1);
for i = 1:Nf
    tmpD(i) =size(getfield(Demos(1),field{i}),1);
end
DemosOut = zeros(sum(tmpD),M*N);
tmpidr = 1;
for i = 1:Nf
    tmpidc = 1;
    for j = 1:M
        DemosOut(tmpidr:tmpidr+tmpD(i)-1,tmpidc:tmpidc+N-1) = getfield(Demos(j),field{i});
        tmpidc = tmpidc + N;
    end
    tmpidr = tmpidr + tmpD(i);
end

end

