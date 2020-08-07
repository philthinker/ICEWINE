function [trajOut] = SSpline(pointsIn,dt,velocityScaler,constraint,mode)
%SSpline S-spline motion plan in joint space
%   pointsIn: N x D, route points
%   dt: scalar, time step
%   velocityScaler, (0,1] scalar, the velocity scaler
%   constraint: 2 x D, constraints
%   |   row 1: maximujm magnitude X
%   |   row 2: minimum magnitude X
%   mode: integer, reserved for future use

if nargin < 5
    mode = 0;
end

if mode == 0
    %% Point-to-point motion plan.
    %   q(t) = X(1-cos(wt))
    %   dq(t) = wXsin(wt)
    %   ddq(t) = wwXcos(wt)
    %   dddq(t) = -wwwXsin(wt)
    %   constraint: 1 x D, constraint of X
    Xmax = constraint(1,:)*0.9; % For safety
    Xmin = constraint(2,:);
    D = size(pointsIn,2);
    i = 2;
    % Maximum magnitude check
    while i <= size(pointsIn,1) && i <=10
%         disp(i);
        if any(abs(pointsIn(i,:) - pointsIn(i-1,:))/2 > Xmax)
            tmpPoints = [pointsIn(1:i-1,:); pointsIn(i-1,:) + (pointsIn(i,:)-pointsIn(i-1,:))/2; pointsIn(i:end,:)];
            pointsIn = tmpPoints;
%             disp('find');
        else
            i = i + 1;
        end
%         disp(pointsIn);
    end
    N = size(pointsIn,1);
    trajs = cell(1,N);
    w = min(max(velocityScaler,0.01),1) * pi/2;
    trajs{1} = pointsIn(1,:);
    for i = 2:N
        % Point to point
        X = (pointsIn(i,:) - pointsIn(i-1,:))/2;
        % Minimum magnitude check
        if all(abs(X) < Xmin)
            trajs{i} = pointsIn(i,:);
        else
            q0 = pointsIn(i-1,:);
            t = (0 : dt : pi/w)';
            trajs{i} = zeros(length(t),D);
            for j = 1:D
                trajs{i}(:,j) = q0(j) + X(j)*(1-cos(w*t));
            end
        end
    end
    trajOut = cell2matrix(trajs,1);
else
    trajOut = pointsIn;
end

end

