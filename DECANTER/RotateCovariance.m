%RotateCovariance

axang = [0,0,1,pi/4];
R = axang2rotm(axang);
R = R([1,2],[1,2]);

Mu = gmm.Mu([1,2],:);
Sigma = gmm.Sigma([1,2],[1,2],:);Sigma2 = Sigma;
for i = 1:size(Sigma,3)
    Sigma2(:,:,i) = R * Sigma(:,:,i);
end

figure; hold on;
plotGMM2SC(Mu, Sigma, [.5 .5 .5],.6);
plotGMM2SC(Mu, Sigma2, [.4 .4 .8],.6);
set(gca,'xtick',[],'ytick',[]); axis equal; axis square;
xlabel(('$x_1$'),'interpreter','latex','fontsize',18);
ylabel(('$x_2$'),'interpreter','latex','fontsize',18);
