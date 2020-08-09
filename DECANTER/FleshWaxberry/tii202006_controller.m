%tii202006_controller

%% Pre - ret - quat
%{
controller = CDynamics(0.001);
controller.K = sqrt(100) * eye(6);
controller.D = sqrt(80) * eye(6);

N = 90/controller.dt;
i = 1;
q0 = Demos_pre(1).q(:,1);
qt = quatnormalize(q0')';
omegat = [0,0,0]';
hatq = Data_pre_q(1).expData_q;
q = zeros(4,N);
hatqt = q;
counter = 1; fps_counter = 1; contrHz = 10;
while i <= N
    if fps_counter >= (1.0/contrHz)/controller.dt
        fps_counter = 0;
        if counter < size(hatq,2)
            counter = counter + 1;
        end
    end
    [qt,omegat,~] = controller.quatDynamics(qt,omegat,hatq(:,counter));
    q(:,i) = qt;
    hatqt(:,i) = hatq(:,counter);
    i = i + 1;
    fps_counter = fps_counter + 1;
end

figure;
t = (1:N)*controller.dt - controller.dt;
ylabels = {'qw','qx','qy','qz'};
for j = 1:4
    subplot(4,1,j);
    plot(t, hatqt(j,:),'b');
    hold on;
    plot(t, q(j,:),'r');
    ylabel(ylabels{j});
    xlabel('t(s)');
    grid on;
end
%}

%% Pre - ret

% Cartesian input with time query
%{
% Controller
stiffness = 200;
damping = 100;
controller_pre = CDynamics(0.001);
controller_pre.K(1:3,1:3) = stiffness * eye(3);
controller_pre.K(4:6,4:6) = sqrt(stiffness/100) * eye(3);
controller_pre.D(1:3,1:3) = damping * eye(3);
controller_pre.D(4:6,4:6) = sqrt(damping/100) * eye(3);
% Retrieve one by one
timeout = 90;
N = timeout/controller_pre.dt;
controll_freq = 10;
for m = 1:M
    % System param.
    i = 1; counter = 1; fps_counter = 1;
    % Data storation
    hat_p = (Data_pre_w(m).query_frame(2).A(2:4,2:4))' * (Data_pre_w(m).expData_p - Data_pre_w(m).query_frame(2).b(2:4,1));
    hat_q = Data_pre_q(m).expData_q;
    p_t = (Demos_pre(m).R(:,:,2))' * (Demos_pre(m).data(2:4,1) - Demos_pre(m).p(:,2));
    q_t = quatRegulate( Demos_pre(m).q(:,1) );
    dp_t = zeros(3,1); omega_t = zeros(3,1); ddp_t = zeros(3,1); domega_t = zeros(3,1);
    p = zeros(3,N); q = zeros(4,N); dp = zeros(3,N); omega = zeros(3,N); ddp = zeros(3,N); domega = zeros(3,N);
    hat_p_c = p; hat_q_c = q;
    % Control loop
    while i <= N
        if fps_counter >= (1.0/controll_freq)/controller_pre.dt
            fps_counter = 0;
            if counter < size(hat_p,2)
                counter = counter + 1;
            end
        end
        [p_t,dp_t,ddp_t] = controller_pre.euclDynamics(p_t,dp_t,hat_p(:,counter));
        [q_t,omega_t,domega_t] = controller_pre.quatDynamics(q_t,omega_t,hat_q(:,counter));
        p(:,i) = p_t; dp(:,i) = dp_t; ddp(:,i) = ddp_t;
        q(:,i) = q_t; omega(:,i) = omega_t; domega(:,i) = omega_t;
        hat_p_c(:,i) = hat_p(:,counter);
        hat_q_c(:,i) = hat_q(:,counter);
        i = i + 1;
        fps_counter = fps_counter + 1;
    end
    % Storation
    Data_pre_w(m).expData_hatp = hat_p;
    Data_pre_w(m).expData_hatp_c = hat_p_c;
    Data_pre_w(m).expData_simp = p;
    Data_pre_w(m).expData_simdp = dp;
    Data_pre_w(m).expData_simddp = ddp;
    Data_pre_q(m).expData_hat_q_c = hat_q_c;
    Data_pre_q(m).expData_simq = q;
    Data_pre_q(m).expData_omega = omega;
    Data_pre_q(m).expData_domega = domega;
    % Figure
    %{
    % Plot the x y z
    figure;
    ylabels = {'x(m)','y(m)','z(m)'};
    t = (1:N)*controller_pre.dt - controller_pre.dt;
    for j = 1:3
        subplot(3,1,j);
        plot(t,hat_p_c(j,:),'b');
        hold on;
        plot(t,p(j,:),'r');
        grid on;
        ylabel(ylabels{j});
        xlabel('t(s)');
    end
    %}
    % Plot the quat
    figure;
    ylabels = {'qw','qx','qy','qz'};
    t = (1:N)*controller_pre.dt - controller_pre.dt;
    for j = 1:4
        subplot(4,1,j);
        plot(t,hat_q_c(j,:),'b');
        hold on;
        plot(t,q(j,:),'r');
        grid on;
        ylabel(ylabels{j});
        xlabel('t(s)');
    end
end
%}

% Cartesian input with time + z query

% Controller
stiffness = 400;
damping = 90;
controller_pre = CDynamics(0.001);
controller_pre.K(1:3,1:3) = stiffness * eye(3);
controller_pre.K(4:6,4:6) = sqrt(stiffness) * eye(3);
controller_pre.D(1:3,1:3) = damping * eye(3);
controller_pre.D(4:6,4:6) = sqrt(damping) * eye(3);
% Retrieve one by one
timeout = 100;
N = timeout/controller_pre.dt;
controll_freq = 10;
for m = 1:M
    % System param.
    i = 1; counter = 1; fps_counter = 1;
    % Data storation
    hat_p = (Data_pre_w(m).query_frame(2).A(2:4,2:4))' * (Data_pre_w(m).expData_p - Data_pre_w(m).query_frame(2).b(2:4,1));
%     hat_q = Data_pre_q(m).expData_q;
    p_t = (Demos_pre(m).R(:,:,2))' * (Demos_pre(m).data(2:4,1) - Demos_pre(m).p(:,2));
    q_t = quatRegulate( Demos_pre(m).q(:,1) );
    dp_t = zeros(3,1); omega_t = zeros(3,1); ddp_t = zeros(3,1); domega_t = zeros(3,1);
    p = zeros(3,N); q = zeros(4,N); dp = zeros(3,N); omega = zeros(3,N); ddp = zeros(3,N); domega = zeros(3,N);
    hat_p_c = p; hat_q_c = q;
    % Control loop
    while i <= N
        if fps_counter >= (1.0/controll_freq)/controller_pre.dt
            fps_counter = 0;
            if counter < size(hat_p,2)
                counter = counter + 1;
            end
        end
        [p_t,dp_t,ddp_t] = controller_pre.euclDynamics(p_t,dp_t,hat_p(:,counter));
        % Compute current hat_q w.r.t. p_t(3)
        [hat_eta_t,~] = policy_pre_quat.GMR(p_t(3,1));
        hat_q_t = policy_pre_quat.expmap(hat_eta_t);
        [q_t,omega_t,domega_t] = controller_pre.quatDynamics(q_t,omega_t,hat_q_t);
        p(:,i) = p_t; dp(:,i) = dp_t; ddp(:,i) = ddp_t;
        q(:,i) = q_t; omega(:,i) = omega_t; domega(:,i) = omega_t;
        hat_p_c(:,i) = hat_p(:,counter);
        hat_q_c(:,i) = hat_q_t;
        i = i + 1;
        fps_counter = fps_counter + 1;
    end
    % Storation
    Data_pre_w(m).expData_hatp = hat_p;
    Data_pre_w(m).expData_hatp_c = hat_p_c;
    Data_pre_w(m).expData_simp = p;
    Data_pre_w(m).expData_simdp = dp;
    Data_pre_w(m).expData_simddp = ddp;
    Data_pre_q(m).expData_hat_q_c = hat_q_c;
    Data_pre_q(m).expData_simq = q;
    Data_pre_q(m).expData_omega = omega;
    Data_pre_q(m).expData_domega = domega;
    % Figure
    % Plot the x y z
    figure;
    ylabels = {'x(m)','y(m)','z(m)'};
    t = (1:N)*controller_pre.dt - controller_pre.dt;
    for j = 1:3
        subplot(3,1,j);
        plot(t,hat_p_c(j,:),'b');
        hold on;
        plot(t,p(j,:),'r');
        grid on;
        ylabel(ylabels{j});
        xlabel('t(s)');
    end
    % Plot the quat
    figure;
    ylabels = {'qw','qx','qy','qz'};
    t = (1:N)*controller_pre.dt - controller_pre.dt;
    for j = 1:4
        subplot(4,1,j);
        plot(t,hat_q_c(j,:),'b');
        hold on;
        plot(t,q(j,:),'r');
        grid on;
        ylabel(ylabels{j});
        xlabel('t(s)');
    end
end
%}

%% Pre - gen
