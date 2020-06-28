%joint_impedance_control

%% Constants
radius = 0.05;
vel_max = 0.25;
acceleration_time = 2.0;
run_time = 20.0;

k_gains = [600,600,600,600,250,150,50]';
d_gains = [50,50,50,50,30,25,15]';

%% Variables
v = 0.0;
angle = 0.0;
t = 0.0;

initial_pose = eye(4);
delta_y = 0;
delta_z = 0;

%% Figure drawnow
% figure_counter_v = 0;
% figure_counter_angle = 0;
% figure_counter_pose = 0;

%% Control Loop

while true
    t = t + 0.001;
    if v < vel_max && t < run_time
        v = v + 0.001*abs(vel_max/acceleration_time);
    end
    if v > 0.0 && t > run_time
        v = v - 0.001*abs(vel_max/acceleration_time);
    end
    v = max(v,0.0);
    v = min(v,vel_max);
%     figure_counter_v = figure_counter_v + 1;
%     if figure_counter_v >= 49
%         scatter(t,v,'b','filled'); hold on; grid on; axis([0,run_time+acceleration_time,0,vel_max]); drawnow;
%         figure_counter_v = 0;
%     end
    angle = angle + 0.001*v/radius;
    if angle > 2*pi
        angle = angle - 2*pi;
    end
%     figure_counter_angle = figure_counter_angle + 1;
%     if figure_counter_angle >= 24
%         scatter(t,angle,'b','filled'); hold on; grid on; axis([0,run_time+acceleration_time,0,2*pi]); drawnow;
%         figure_counter_angle = 0;
%     end
    delta_y = radius * (1-cos(angle));
    delta_z = radius * sin(angle);
    pose_desired = initial_pose;
    pose_desired(2,4) = pose_desired(2,4) + delta_y;
    pose_desired(3,4) = pose_desired(3,4) + delta_z;
    figure_counter_pose = figure_counter_pose + 1;
%     if figure_counter_pose >= 49
%         scatter3(pose_desired(1,4),pose_desired(2,4),pose_desired(3,4),'b','filled'); 
%         view(3);  hold on; grid on; axis([-1,1,0,0.1,-0.05,0.05]); drawnow;
%         figure_counter_pose = 0;
%     end
    if t >= run_time + acceleration_time
        break
    end
end