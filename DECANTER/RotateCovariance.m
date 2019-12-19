%RotateCovariance

%% Rotate Covariances for Y. Chen

% axang = [0,0,1,pi/4];
% R = axang2rotm(axang);
% R = R([1,2],[1,2]);
% 
% Mu = gmm.Mu([1,2],:);
% Sigma = gmm.Sigma([1,2],[1,2],:);Sigma2 = Sigma;
% for i = 1:size(Sigma,3)
%     Sigma2(:,:,i) = R * Sigma(:,:,i);
% end
% 
% figure; hold on;
% plotGMM2SC(Mu, Sigma, [.5 .5 .5],.6);
% plotGMM2SC(Mu, Sigma2, [.4 .4 .8],.6);
% set(gca,'xtick',[],'ytick',[]); axis equal; axis square;
% xlabel(('$x_1$'),'interpreter','latex','fontsize',18);
% ylabel(('$x_2$'),'interpreter','latex','fontsize',18);

%% Test computeQuiver3

N = 20;
theta = linspace(0,pi/2,N);
ax = [1,0,0];

x = linspace(1,10,N)';
y = linspace(1,1,N)';
z = linspace(1,1,N)';

figure;

for i = 1:N
    [~,U,V,W] = computeQuiver3(x(i),y(i),z(i),theta(i),ax);
    quiver3(x(i),y(i),z(i),U(1),U(2),U(3),'Color',[1,0,0]);
    hold on;
    quiver3(x(i),y(i),z(i),V(1),V(2),V(3),'Color',[0,1,0]);
    quiver3(x(i),y(i),z(i),W(1),W(2),W(3),'Color',[0,0,1]);
end
axis equal;
xlabel('x');ylabel('y');zlabel('z');
grid on;