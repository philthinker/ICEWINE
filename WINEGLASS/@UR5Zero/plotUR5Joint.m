function [] = plotUR5Joint(obj,trajJoint, T)
%plotUR5Joint Plot the joints of Panda robot
%   trajJoint: N x 6, joint traj.
%   T: scalar, the end of time series (default: 1)
%   @UR5Zero

if nargin<3
    T = 1;
end
N = size(trajJoint,1);
t = linspace(0,T,N);

figure;
for i = 1:6
    subplot(6,1,i);
    plot(t,trajJoint(:,i));
    grid on;
end

end

