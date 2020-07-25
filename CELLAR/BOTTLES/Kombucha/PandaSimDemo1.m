%PandaSimDemo1
%   A template for simulating the control loop of Franka Panda
%
%   Haopeng Hu
%   2020.06.20
%   All rights reserved

%% External variable

desired_mass = 0.0;
target_mass = 1.0;
k_p = 1.0;
k_i = 1.0;
filter_gain = 0.001;

robot = PandaSimOne();

initial_tau_ext = zeros(1,7);
tau_error_integral = zeros(1,7);

initial_position = zeros(1,3);
time = 0.0;

%% Simulated feedback

gravity_array = zeros(1,7);
initial_tau_measure = zeros(1,7);
initial_gravity = gravity_array;
inital_tau_ext = initial_tau_measure - initial_gravity;
get_position = zeros(1,3);

%% Control loop

while true
    time = time + robot.toSec();
    jacobian = zeros(6,7);
    tau_measured = zeros(1,7);
    gravity = zeros(1,7);
    break
end
