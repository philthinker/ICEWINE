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
dt = 0.001;

%% Empirical maximum discrepancy interpolation (Not recommended)
% Safe but very slow and jittering
% The last choice.
%{
delta_q_max = [4e-4, 4e-4, 6e-4, 6e-4, 8e-4, 8e-4, 8e-4]/4; % Never forget its RT feature
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
%}

%% S-spline point-to-point motion plan

ssplineConstraint_max = [pi/8, pi/8, pi/6, pi/6, pi/6, pi/4, pi/4];
ssplineConstraint_min = [5e-4, 5e-4, 5e-4, 1e-3, 1e-3, 2e-3, 2e-3];
trajOut = SSpline(trajIn,dt,0.5, [ssplineConstraint_max; ssplineConstraint_min]);
NOut = size(trajOut,1);

%% 

end
