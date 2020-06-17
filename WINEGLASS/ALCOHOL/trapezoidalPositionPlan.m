function [trajOut, vtrajOut, atrajOut, jtrajOut] = trapezoidalPositionPlan(pointsIn,dt,velocityScaler,constraint,mode)
%trapezoidalPositionPlan Trapezoidal velocity motion plan for given
%positions and maximum acceleration or jerk
%   pointsIn: N x D, route points
%   dt: scalar, time step
%   velocityScaler, (0,1] scalar, the velocity scaler
%   constraint: 2 x D or 3 x D, constraints
%   |   row 1: maximum velocity
%   |   row 2: maximum acceleration
%   |   row 3: maximum jerk (optional)
%   mode: integer, reserved for future use

if nargin < 5
    mode = 0;
end

DO = size(constraint,1);
a = min(max(velocityScaler,0.01),1);

if mode == 0
    %% Point  to point motion plan
else
    trajOut = pointsIn;
    vtrajOut = [];
    atrajOut = [];
    jtrajOut = [];
end

end

