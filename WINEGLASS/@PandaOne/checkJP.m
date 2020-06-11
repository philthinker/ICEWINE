function [flag, error_id] = checkJP(obj, traj)
%checkJP Check the joint positin trajectory to meet the constraints
%   traj: N x 7, joint position trajectory (Unit: rad)
%   flag: boolean, true for no violation
%   error_id: integer, indicating the violation item
%   |   0: No violation
%   |   1: Joint position violation
%   |   2: Joint velocity violation or joint discrepency violation
%   |   3: Joint acceleration violation
%   |   4: Joint jerk violation
%   @PandaOne
%
%   q_min < q < q_max
%   -dq_max < dq < dq_max
%   -ddq_max < ddq < ddq_max
%   -dddq_max < dddq < dddq_max
%   dq(0) = 0
%   ddq(0) = 0
%   dq(end) = 0
%   ddq(end) = 0
%
%   Note that the time step is always 1ms, i.e. 0.001s

flag = true;
error_id = 0;
% dt = 0.001;

% q_max, q_min, dq_max, ddq_max, dddq_max
constraint = obj.JointConstraint;
% q1, q2, q3, q4, q5, q6, q7
constraint_deltaJP_empirical = [6e-4, 6e-4, 8e-4, 8e-4, 8e-4, 1e-3, 1e-3];

% traj
% Check whether q violates its constraint
flag = flag && all(max(traj,[],1) < constraint(1,:)) && all(min(traj,[],1) > constraint(2,:));
if ~flag
    error_id = 1;
    return
end
% Check whether q violates the empirical constraint
dtraj = abs(traj(2:end,:) - traj(1:end-1,:));
flag = flag && all(max(dtraj,[],1) < constraint_deltaJP_empirical);
if ~flag
    error_id = 2;
    return
end
%{
% Check whether dq violates its constraint
dtraj = [zeros(1,7); (traj(2:end,:) - traj(1:end-1,:))/dt];
flag = flag && all(max(abs(dtraj),[],1) < constraint(3,:));
if ~flag
    error_id = 2;
    return
end
% Check whether ddq violates its constraint
ddtraj = [zeros(1,7); (dtraj(2:end,:) - dtraj(1:end-1,:))/dt];
flag = flag && all(max(abs(ddtraj),[],1) < constraint(4,:));
if ~flag
    error_id = 3;
    return
end
% Check whether dddq violates its constraint
dddtraj = [zeros(1,7); (ddtraj(2:end,:) - ddtraj(1:end-1,:))/dt];
flag = flag && all(max(abs(dddtraj),[],1) < constraint(5,:));
if ~flag
    error_id = 4;
    return
end
%}

end

