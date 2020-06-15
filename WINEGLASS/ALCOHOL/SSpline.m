function [trajOut] = SSpline(pointsIn,dt,constraint,mode)
%SSpline S-spline motion plan in joint space
%   pointsIn: N x D, route points
%   dt: scalar, time step
%   constraint: NC x D, constraints
%   mode: integer, reserved for future use

if nargin < 3
    mode = 0;
end

if mode == 0
    %% Point-to-point motion plan with maximum jerk constraint
    % We assume the constraint is of the form: [qmax, qmin, dqmax,  dqmin,
    % ddqmax, ddqmin, dddqmax, dddqmin]'.
    
end

end

