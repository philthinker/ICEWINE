function [trajOut, NOut] = interpJP(obj,trajIn)
%interpJP Joint position interpolation
%   trajIn: N x 7, joint position trajectory
%   trajOut: NOut x 7, joint position trajectory
%   NOut: integer, number of data in trajOut
%   @PandaOne
%
%   We provide several motion planning methods.
%   Uncomment what you want.
%   Note we ought to always assume that the initial
%   and final velocity and acceleration are ZERO.

N = size(trajIn,1);
% q_max, q_min, dq_max, ddq_max, dddq_max
constraint = obj.JointConstraint;

%% Empirical maximum discrepancy interpolation
% Safe but very slow!
% The last choice.
delta_q_max = [4e-4, 4e-4, 6e-4, 6e-4, 8e-4, 8e-4, 8e-4]/2; % Never forget its RT feature
tmpTrajs = cell(1,N-1);
for i = 2:N
    tmpN = max(ceil(abs(trajIn(i,:) - trajIn(i-1,:))./delta_q_max));
    if tmpN == 0
        % The same route point
        tmpTrajs{i-1} = trajIn(i-1,:);
    elseif tmpN == 1
        % One step is allowed
        tmpTrajs{i-1} = trajIn(i-1,:);
    else
        delta_q = (trajIn(i,:) - trajIn(i-1,:))/tmpN;
        tmpTrajs{i-1} = [trajIn(i-1,:) ; ...
            (1:tmpN-1)'*delta_q+trajIn(i-1,:)];
    end
end
trajOut = trajs2traj(tmpTrajs);
NOut = size(trajOut,1);

%% Simple point-to-point maximum jerk iterative interpolation
%{
tmpTrajs = cell(1,N-1);
tmpN = 0;
for i = 2:N
    
end
trajOut = trajs2traj(tmpTrajs);
NOut = size(trajOut,1);
%}

%% 

end

function [traj] = trajs2traj(trajs)
    %trajs2traj Merge the data in cells into one matrix
    %   trajs: 1 x M cell
    %   traj: NOut x 7
    M = length(trajs);
    tmpN = zeros(1,M);
    for i = 1:M
        tmpN(i) = size(trajs{i},1);
    end
    NOut = sum(tmpN);
    traj = zeros(NOut,7);
    tmpIndex = 1;
    for i = 1:M
        traj(tmpIndex : tmpIndex+tmpN(i)-1,:) = trajs{i};
        tmpIndex = tmpIndex + tmpN(i);
    end
end
