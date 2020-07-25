%generate_joint_velocity_test

%% External variables

time_max = 1.0;
omega_max = 1.0;
time = 0.0;

%% Control loop: Robot.control

% Control signal
velocities = zeros(10000,7);

period = 0.001;   % Duration object
i = 0;               % Iteration counter
while true
    i = i + 1;
    %% Control law
    time = time + period;
    cycle = floor((-1)^(time - mod(time,time_max)/time_max));
    omega = cycle * omega_max/2.0 * (1.0 - cos(2.0 * pi/time_max * time));
    velocities(i,:) = [0.0, 0.0, 0.0, omega, omega, omega, omega];
    %% Terminal condition
    if time >= 2*time_max
        break
    end
end
[velocities,N] = curtialTial(velocities);

%% Data out

t = (1:N)*0.001;
plot(t,velocities);
grid on;
legend;
