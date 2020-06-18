%PandaSimDemo
%   A template for simulating the control loop of Franka Panda
%
%   Haopeng Hu
%   2020.06.17
%   All rights reserved

%% Init. PandaSim

panda = PandaSim(0.001,PandaSim.CartesianVelocities);

%% External variables

time_max = 4.0;
v_max = 0.1;
angle = pi/4;
time = 0.0;

%% Control loop: Robot.control
while true
    %% Control law
    time = time + panda.toSec();
    cycle = floor( (-1.0)^((time - mod(time, time_max ) )/time_max));
    v = cycle * v_max/2.0 * (1.0 - cos(2.0*pi/time_max * time));
    v_x = cos(angle) * v;
    v_z = -sin(angle) * v;
    output = [v_x, 0.0, v_z, 0.0, 0.0, 0.0];
    panda = panda.simUpdate(output);
    %% Terminal condition
    if time >= 2*time_max
        [panda,N] = panda.simTerminate();
        break
    end
end

%% Data out

t = (1:N)'*panda.toSec();

figure;
plot(t, [panda.motions(:,1),panda.motions(:,3)]);
grid on;
legend;