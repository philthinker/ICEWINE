%DrawSemicircle

% Peter Corke's Robotics Toolbox is required
% Haopeng Hu
% 2020.06.01 Happy Children's Day

R = 100;
N = 9;
NN = 100;
t = linspace(0,1,N);
theta = linspace(0,pi,NN);
T = repmat(SO3P2SE3(eye(3),[R,0,0]'),[1,1,N]);

figure;

for i = 1:N
    T(:,:,i) = axang2tform([0,-1,0,t(i)*pi]) * T(:,:,i);
    trplot(T(:,:,i), ...
        'arrow', ...
        'thick', 1, ...
        'length', 20, ...
        'width', 0.5, ...
        'color', 'k', ...
        'rgb', ...
        'axis', [-130,130,-130,130,-50,210]);
    hold on;
end
plot3(R*cos(theta), zeros(1,NN), R*sin(theta),'k');
view([49,25]);