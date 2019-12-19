function [] = plotPandaJoint(obj,trajJoint, T)
%plotPandaJoint Plot the joints of Panda robot
%   trajJoint: N x 7, joint traj.
%   T: scalar, the end of time series (default: 1)
%   @PandaZero

if nargin<3
    T = 1;
end
N = size(trajJoint,1);
t = linspace(0,T,N);

figure;
for i = 1:7
    subplot(7,1,i);
    plot(t,trajJoint(:,i));
    grid on;
end

end

