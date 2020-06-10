%DrawStraightLine

% Peter Corke's Robotics Toolbox is required
% Haopeng Hu
% 2020.06.01 Happy Children's Day

L = 400; % 400mm
N = 5; % Num. of points
t = linspace(0,1,N);
T = repmat(eye(4),[1,1,N]);
T(1,4,:) = permute(L * t,[3,1,2]);

figure;
for i = 1:N
    trplot(T(:,:,i), ...
        'arrow', ...
        'thick', 1, ...
        'length', 60, ...
        'width', 0.5, ...
        'color', 'k', ...
        'rgb', ...
        'axis', [-50,550,-300,300,-300,300]);
    hold on;
end
plot3([0,400],[0,0],[0,0],'k');
view([49,25]);