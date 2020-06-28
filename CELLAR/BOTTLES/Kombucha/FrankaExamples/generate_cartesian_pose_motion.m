%generate_cartesian_pose_test
%
%   Haopeng Hu
%   2020.06.18
%   All rights reserved

%% Init. PandaSim

panda = PandaSim(0.001,PandaSim.CartesianPose);

%% External variables

initial_pose = zeros(1,16);
time = 0.0;

%% Control loop: Robot.control
O_T_EE_c = flattenSE3(eye(4));
while true
%     panda.iCounter = panda.iCounter + 1;
    %% Control law
    time = time + panda.getPeriod();
    if time == 0.0
        initial_pose = O_T_EE_c;
    end
    kRadius = 0.3;
    angle = pi/4 * (1-cos(pi/5 * time));
    delta_x = kRadius * sin(angle);
    delta_z = kRadius * (cos(angle) - 1);
    new_pose = initial_pose;
    new_pose(13) = new_pose(13) + delta_x;
    new_pose(15) = new_pose(15) + delta_z;
    panda = panda.simUpdate(new_pose);
%     panda.motions(panda.iCounter,:) = new_pose;
    %% Terminal condition
    if time >= 10.0
        [panda,N] = panda.simTerminate();
        break
    end
end

%% Data out

t = (1:N)'*panda.getPeriod();

figure;
plot(t, [panda.motions(:,13),panda.motions(:,15)]);
grid on;
legend;

figure;
plot3(panda.motions(:,13), panda.motions(:,14), panda.motions(:,15));
grid on;
view(3);
