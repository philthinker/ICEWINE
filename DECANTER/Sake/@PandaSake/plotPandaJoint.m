function [] = plotPandaJoint(obj,trajJoint,T,dt)
%plotPandaJoint Plot the joints of Panda robot
%   trajJoint: 7 x N, joint traj.
%   T: scalar, the end of time series (default: 1, 0 for no scaling)
%   dt: scalar, time step (default: 0.001)
%   @PandaSake

N = size(trajJoint,2);

if nargin<3
    T = 1;
    t = linspace(0,T,N);
else
    if T == 0
        % dt must be specified when T == 0
        if nargin < 4
            % Otherwise dt is set to default value
            dt = 1e-3;
        end
        t = (0:N-1)*dt;
    else
        t = linspace(0,T,N);
    end
end

for i = 1:7
    subplot(7,1,i);
    plot(t,trajJoint(i,:));
    grid on;
    hold on;
end

end

