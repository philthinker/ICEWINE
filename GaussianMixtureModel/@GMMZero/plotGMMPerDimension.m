function [] = plotGMMPerDimension(obj,demos,color,valAlpha)
%plotGMMPerDimension Display the parameters of the GMM dimension by
%dimension in one figure
%   demos: 1 x M cell, demos with N x D+1 data
%   color: 3 x 1 array representing the RGB color to use for the display.
%   valAlpha: transparency factor (optional).
%   @GMMZero

figure;
M = length(demos);
D = size(demos{1},2)-1; % The very left column is the time series

for i = 1:D
    subplot(D,1,i);
    for j = 1:M
        % Plot the demos
        plot(demos{j}(:,1),demos{j}(:,i+1),'Color',[0.36,0.36,0.36]);
        hold on;
    end
    % scatter the mean values
%     if nargin < 4
%         % No alpha specified, scatter without transparency
%         scatter(obj.Mu(:,1),obj.Mu(:,i+1),20,color);
%     else
%         % alpha specified, scatter with transparency
%         scatter(obj.Mu(:,1),obj.Mu(:,i+1),20,color,'filled');
%     end
    % patch the mean values as well as covariances
    tmpSigma = obj.Sigma([1,i+1],[1,i+1],:);
    if nargin < 4
        % No Transparency
        obj.plotGMM2SCPro(obj.Mu(:,[1 i+1]),tmpSigma,color);
    else
        % Transparency
        obj.plotGMM2SCPro(obj.Mu(:,[1 i+1]),tmpSigma,color,valAlpha);
    end
    grid on;
    axis([demos{1}(1,1), demos{1}(end,1), -inf,inf]);
end

end

