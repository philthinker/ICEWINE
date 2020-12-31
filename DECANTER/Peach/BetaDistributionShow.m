%BetaDistributionShow

% Refer to:
% https://blog.csdn.net/philthinker/article/details/111999552

%% Beta func.

% s = linspace(1,3,1000);
% t = linspace(1,3,1000);
% 
% [S,T] =meshgrid(s,t);
% B = beta(S,T);
% 
% figure;
% imagesc(s,t,B);
% colorbar;
% xlabel('s'); ylabel('t');

%% Beta distri.

N = 1000;
x = linspace(0,1,N);

s = [0.5,  1,  2.5]; 
t = [0.5,  1,  2.5];
M = length(s);

p = zeros(M*M,N);

count = 1;
for i = 1:M
    for j = 1:M
        p(count,:) = (1/beta(s(i),t(j))) .* (x.^(s(i)-1)) .* ((1-x).^(t(j)-1));
        count = count + 1;
    end
end

figure;
count = 1;
for i = 1:M
    for j = 1:M
        subplot(M,M,count);
        plot(x,p(count,:));
        xlabel('x'); ylabel('p'); title(strcat('s=',num2str(s(i)),' t=',num2str(t(j))));
        count = count + 1;
    end
end


