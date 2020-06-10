function [flag, error_id] = checkJV(obj, traj)
%checkJV Check the joint velocity trajectory to meet the constraints
%   traj: N x 7, joint position trajectory (Unit: rad/s)
%   flag: boolean, true for no violation
%   error_id: integer, indicating the violation item
%   |   0: No violation
%   |   2: Joint velocity violation
%   |   3: Joint acceleration violation
%   |   4: Joint jerk violation
%   @PandaOne
%
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
dt = 0.001;

% q_max, q_min, dq_max, ddq_max, dddq_max
constraint = obj.JointConstraint(3:end,:);

% dtraj, ddtraj, dddtraj
% Check whether dq violates its constraint
flag = flag && all(traj(1,:) == 0) && all(traj(end,:) == 0);
flag = flag && all(max(abs(traj),[],1) < constraint(1,:));
if ~flag
    error_id = 2;
    return
end
% Check whether ddq violates its constraint
dtraj = [zeros(1,7); (traj(2:end,:) - traj(1:end-1,:))/dt];
flag = flag && all(max(abs(dtraj),[],1) < constraint(2,:));
if ~flag
    error_id = 3;
    return
end
% Check whether dddq violates its constraint
ddtraj = [zeros(1,7); (dtraj(2:end,:) - dtraj(1:end-1,:))/dt];
flag = flag && all(max(abs(ddtraj),[],1) < constraint(3,:));
if ~flag
    error_id = 4;
    return
end

end

