function [] = plot3PandaCartesian(obj,trajCartesian)
%plot3PandaCartesian Plot the positions of Panda robot
%   trajCartesian: 4 x 4 x N, SE3 trajectory or N x 3, position trajectory
%   @PandaZero

if size(trajCartesian,2) == 4
    % trajCartesian is a SE3 trajectory
    traj = permute(trajCartesian(1:3,4,:),[3,1,2]);
elseif size(trajCartesian,2) == 3
    % trajCartesian is a position trajectory
    traj = trajCartesian;
end

figure;
plot3(traj(:,1),traj(:,2),traj(:,3));
grid on; axis equal;

end

