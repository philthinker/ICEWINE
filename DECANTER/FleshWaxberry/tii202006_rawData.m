%tii202006_rawData

%% Read from csv

%{
% pre = [];
% pre.otee = zeros(1,16);
% cis = [];
% cis.otee = zeros(1,16);
% cis.kfk = zeros(1,6);
% cis.ofk = zeros(1,6);
% 
% Demo = [];
% Demo.pre = pre;
% Demo.cis = cis;
% 
% M = 9;
% Demo = repmat(Demo,[1,M]);

% for i = 1:M
%     Demo(i).pre.otee = readmatrix(strcat('DECANTER\FleshWaxberry\Data\07-21-K\waxberry_',...
%         'pre0', int2str(i), '_OTEE.csv'));
%     % Get rid of the last column
%     Demo(i).pre.otee = Demo(i).pre.otee(:,1:end-1);
%     Demo(i).cis.otee = readmatrix(strcat('DECANTER\FleshWaxberry\Data\07-21-K\waxberry_',...
%         'cis0', int2str(i), '_OTEE.csv'));
%     Demo(i).cis.otee = Demo(i).cis.otee(:,1:end-1);
%     Demo(i).cis.kfk = readmatrix(strcat('DECANTER\FleshWaxberry\Data\07-21-K\waxberry_',...
%         'cis0', int2str(i), '_KFK.csv'));
%     Demo(i).cis.kfk = Demo(i).cis.kfk(:,1:end-1);
%     Demo(i).cis.ofk = readmatrix(strcat('DECANTER\FleshWaxberry\Data\07-21-K\waxberry_',...
%         'cis0', int2str(i), '_OFK.csv'));
%     Demo(i).cis.ofk = Demo(i).cis.ofk(:,1:end-1);
% end


% % Convert to unit quaternion
% for i = 1:M
%     Demo(i).pre.se3 = fold2SE3(Demo(i).pre.otee);
%     Demo(i).cis.se3 = fold2SE3(Demo(i).cis.otee);
%     Demo(i).pre.pq = SE3toPQ(Demo(i).pre.se3);
%     Demo(i).cis.pq = SE3toPQ(Demo(i).cis.se3);
% end



%}

% % ¡®lead0722¡¯

%% Plot raw data

%{
figure;
hold on;
for i = 1:M
    plot3(Demo(i).pre.otee(:,13), Demo(i).pre.otee(:,14), Demo(i).pre.otee(:,15));
end
grid on; axis equal;
view(3);

figure;
hold on;
for i = 1:M
    plot3(Demo(i).cis.otee(:,13), Demo(i).cis.otee(:,14), Demo(i).cis.otee(:,15));
end
grid on; axis equal;
view(3);

figure;
dt = 0.05;
for i = 1:6
    subplot(6,1,i);
    for m = 1:M
        t = (1:size(Demo(m).cis.kfk, 1))*dt - dt;
        plot(t,Demo(m).cis.kfk(:,i));
        hold on;
    end
    grid on;
end

figure;
dt = 0.05;
for i = 1:6
    subplot(6,1,i);
    for m = 1:M
        t = (1:size(Demo(m).cis.ofk, 1))*dt - dt;
        plot(t,Demo(m).cis.ofk(:,i));
        hold on;
    end
    grid on;
end

figure;
for i = 1:4
    subplot(4,1,i);
    for m = 1:M
        t = (1:size(Demo(m).pre.pq, 1))*dt - dt;
        plot(t,Demo(m).pre.pq(:,i+3));
        hold on;
    end
    grid on;
end

figure;
for i = 1:4
    subplot(4,1,i);
    for m = 1:M
        t = (1:size(Demo(m).cis.pq, 1))*dt - dt;
        plot(t,Demo(m).cis.pq(:,i+3));
        hold on;
    end
    grid on;
end
%}
%{
% % Whether the phaset trick works in cis?
figure;
for m = 1:M
    t = (1:size(Demo(m).cis.pq,1))*dt - dt;
    plot(t, Demo(m).cis.pq(:,3));
    hold on;
end
grid on;
%}

%% DTW
%{
% % Only P is needed
tmpCell_pre = cell(1,M); tmpCell_pre_dtw = cell(1,M);
tmpCell_cis = cell(1,M); tmpCell_cis_dtw = cell(1,M);
for m = 1:M
    tmpCell_pre{m} = Demo(m).pre.pq(:,1:3);
    tmpCell_cis{m} = Demo(m).cis.pq(:,1:3);
end
tmpCell_pre_dtw  = iceDTW(tmpCell_pre, 20);
tmpCell_cis_dtw = iceDTW(tmpCell_cis, 20);

figure;
for i = 1:3
    subplot(3,1,i);
    hold on;
    for m = 1:M
        t = (1:size(tmpCell_pre{m},1))*dt-dt;
        plot(t,tmpCell_pre{m}(:,i));
    end
    grid on;
end

figure;
for i = 1:3
    subplot(3,1,i);
    hold on;
    for m = 1:M
        t = (1:size(tmpCell_pre_dtw{m},1))*dt-dt;
        plot(t,tmpCell_pre_dtw{m}(:,i));
    end
    grid on;
end

figure;
for i = 1:3
    subplot(3,1,i);
    hold on;
    for m = 1:M
        t = (1:size(tmpCell_cis{m},1))*dt-dt;
        plot(t,tmpCell_cis{m}(:,i));
    end
    grid on;
end

figure;
for i = 1:3
    subplot(3,1,i);
    hold on;
    for m = 1:M
        t = (1:size(tmpCell_cis_dtw{m},1))*dt-dt;
        plot(t,tmpCell_cis_dtw{m}(:,i));
    end
    grid on;
end

for m = 1:M
    Demo(m).pre.p_dtw = tmpCell_pre_dtw{m};
    Demo(m).cis.p_dtw = tmpCell_cis_dtw{m};
end
%}

%% Prepare for TP-GMM policy learning

% % Policy I: TP-GMM for position with time query

%{
% Demo_test = [];
% Demo_test.tp1.delta_p = [];
% Demo_test.tp1.R = [];
% Demo_test.tp2.delta_p = [];
% Demo_test.tp1.R = [];
% Demo_test = repmat(Demo_test,[1,M]);

for i = 1:M
    % R,p
    Demo(i).pre.R = repmat(eye(3),[1,1,2]);
    Demo(i).pre.p = repmat(zeros(3,1),[1,2]);
    Demo(i).pre.R(:,:,1) = Demo(i).pre.se3(1:3,1:3,1);
    Demo(i).pre.R(:,:,2) = Demo(i).pre.se3(1:3,1:3,end);
    Demo(i).pre.p(:,1) = permute(Demo(i).pre.se3(1:3,4,1),[1,3,2]);
    Demo(i).pre.p(:,2) = permute(Demo(i).pre.se3(1:3,4,end),[1,3,2]);
    % A,b
    Demo(i).pre.A = repmat(eye(4),[1,1,2]);
    Demo(i).pre.b = repmat(zeros(4,1),[1,2]);
    Demo(i).pre.A(:,:,1) = blkdiag(1,Demo(i).pre.R(:,:,1));
    Demo(i).pre.A(:,:,2) = blkdiag(1,Demo(i).pre.R(:,:,2));
    Demo(i).pre.b(:,1) = [0; Demo(i).pre.p(:,1)];
    Demo(i).pre.b(:,2) = [0; Demo(i).pre.p(:,2)];
end

for i = 1:M
    N = size(Demo(i).pre.p_dtw,1);
    Demo(i).pre.tp1p_dtw = zeros(3,N);
    Demo(i).pre.tp2p_dtw = zeros(3,N);
    R1 = Demo(i).pre.R(:,:,1);
    p1 = Demo(i).pre.p(:,1);
    R2 = Demo(i).pre.R(:,:,2);
    p2 = Demo(i).pre.p(:,2);
    for j = 1:N
%         Demo_test(i).tp1.delta_p(1:3,j) = (Demo(i).pre.p_dtw(j,:))' - Demo(i).pre.p(:,1);
%         Demo_test(i).tp1.R = Demo(i).pre.R(:,:,1);
        Demo(i).pre.tp1p_dtw(:,j) = R1' * ((Demo(i).pre.p_dtw(j,:))' - p1);
%         Demo_test(i).tp2.delta_p(1:3,j) = (Demo(i).pre.p_dtw(j,:))' - Demo(i).pre.p(:,2);
%         Demo_test(i).tp2.R = Demo(i).pre.R(:,:,2);
%         Demo_test(i).tp2.data(1:3,j) = Demo_test(i).tp2.R' * Demo_test(i).tp2.delta_p(:,j);
        Demo(i).pre.tp2p_dtw(:,j) = R2' * ((Demo(i).pre.p_dtw(j,:))' - p2);
    end
    Demo(i).pre.tp1p_dtw(abs(Demo(i).pre.tp1p_dtw) < 1e-6) = 0;
    Demo(i).pre.tp2p_dtw(abs(Demo(i).pre.tp2p_dtw) < 1e-6) = 0;
end
%}

% % Policy I: QGMM with z query

%{
for i = 1:M
    N = size(Demo(i).pre.pq,1);
    Demo(i).pre.tp1p = zeros(3,N);
    Demo(i).pre.tp1q = zeros(4,N);
    Demo(i).pre.tp2p = zeros(3,N);
    Demo(i).pre.tp1q = zeros(4,N);
    for j = 1:N
        Demo(i).pre.tp1p(:,j) = Demo(i).pre.R(:,:,1)'*(Demo(i).pre.pq(j,1:3)' - Demo(i).pre.p(:,1));
        Demo(i).pre.tp1q(:,j) = quatProduct( quatconj(Demo(i).pre.pq(1,4:7))', Demo(i).pre.pq(j,4:7)' );
        Demo(i).pre.tp2p(:,j) = Demo(i).pre.R(:,:,2)'*(Demo(i).pre.pq(j,1:3)' - Demo(i).pre.p(:,2));
        Demo(i).pre.tp2q(:,j) = quatProduct( quatconj(Demo(i).pre.pq(end,4:7))', Demo(i).pre.pq(j,4:7)' );
    end
    Demo(i).pre.tp1p(abs(Demo(i).pre.tp1p) < 1e-6) = 0;
    Demo(i).pre.tp1q(abs(Demo(i).pre.tp1q) < 1e-6) = 0;
    Demo(i).pre.tp2p(abs(Demo(i).pre.tp2p) < 1e-6) = 0;
    Demo(i).pre.tp2q(abs(Demo(i).pre.tp2q) < 1e-6) = 0;
end
%}

% % Policy II: TP-GMM for position with time query

%{
for i = 1:M
    % R,p
    Demo(i).cis.R = repmat(eye(3),[1,1,2]);
    Demo(i).cis.p = repmat(zeros(3,1),[1,2]);
    Demo(i).cis.R(:,:,1) = Demo(i).cis.se3(1:3,1:3,1);
    Demo(i).cis.R(:,:,2) = Demo(i).cis.se3(1:3,1:3,end);
    Demo(i).cis.p(:,1) = permute(Demo(i).cis.se3(1:3,4,1),[1,3,2]);
    Demo(i).cis.p(:,2) = permute(Demo(i).cis.se3(1:3,4,end),[1,3,2]);
    % A,b
    Demo(i).cis.A = repmat(eye(4),[1,1,2]);
    Demo(i).cis.b = repmat(zeros(4,1),[1,2]);
    Demo(i).cis.A(:,:,1) = blkdiag(1,Demo(i).cis.R(:,:,1));
    Demo(i).cis.A(:,:,2) = blkdiag(1,Demo(i).cis.R(:,:,2));
    Demo(i).cis.b(:,1) = [0; Demo(i).cis.p(:,1)];
    Demo(i).cis.b(:,2) = [0; Demo(i).cis.p(:,2)];
end

for i = 1:M
    N = size(Demo(i).cis.p_dtw,1);
    Demo(i).cis.tp1p_dtw = zeros(3,N);
    Demo(i).cis.tp2p_dtw = zeros(3,N);
    for j = 1:N
        Demo(i).cis.tp1p_dtw(:,j) = Demo(i).cis.R(:,:,1)'*(Demo(i).cis.p_dtw(j,:)' - Demo(i).cis.p(:,1));
        Demo(i).cis.tp2p_dtw(:,j) = Demo(i).cis.R(:,:,2)'*(Demo(i).cis.p_dtw(j,:)' - Demo(i).cis.p(:,2));
    end
    Demo(i).cis.tp1p_dtw(abs(Demo(i).cis.tp1p_dtw) < 1e-6) = 0;
    Demo(i).cis.tp2p_dtw(abs(Demo(i).cis.tp2p_dtw) < 1e-6) = 0;
end

for i = 1:M
    Demo(i).cis.tp1p_dtw_mm = Demo(i).cis.tp1p_dtw * 1000;
    Demo(i).cis.tp2p_dtw_mm = Demo(i).cis.tp2p_dtw * 1000;
end
%}

% % Policy II: QGMM with z query

%{
for i = 1:M
    N = size(Demo(i).cis.pq,1);
    Demo(i).cis.tp1p = zeros(3,N);
    Demo(i).cis.tp1q = zeros(4,N);
    Demo(i).cis.tp2p = zeros(3,N);
    Demo(i).cis.tp1q = zeros(4,N);
    for j = 1:N
        Demo(i).cis.tp1p(:,j) = Demo(i).cis.R(:,:,1)'*(Demo(i).cis.pq(j,1:3)' - Demo(i).cis.p(:,1));
        Demo(i).cis.tp1q(:,j) = quatProduct( quatconj(Demo(i).cis.pq(1,4:7))', Demo(i).cis.pq(j,4:7)' );
        Demo(i).cis.tp2p(:,j) = Demo(i).cis.R(:,:,2)'*(Demo(i).cis.pq(j,1:3)' - Demo(i).cis.p(:,2));
        Demo(i).cis.tp2q(:,j) = quatProduct( quatconj(Demo(i).cis.pq(end,4:7))', Demo(i).cis.pq(j,4:7)' );
    end
    Demo(i).cis.tp1p(abs(Demo(i).cis.tp1p) < 1e-6) = 0;
    Demo(i).cis.tp1q(abs(Demo(i).cis.tp1q) < 1e-6) = 0;
    Demo(i).cis.tp2p(abs(Demo(i).cis.tp2p) < 1e-6) = 0;
    Demo(i).cis.tp2q(abs(Demo(i).cis.tp2q) < 1e-6) = 0;
end
%}

% % Policy II: GMM for wrench with z query

