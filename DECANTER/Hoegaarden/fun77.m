function [ret] = fun77(X,Y,Z,Q3,Q4,min_color,max_color)
min_color = min_color / 255;
max_color = max_color / 255;
S = 5.*ones(size(X));
N = size(X,1);  % How many rows?

d = [];
for i=1:length(X)
    P = [X(i), Y(i), Z(i)];
    di = norm(cross(Q4 - Q3, P - Q3)) / norm(Q4 - Q3);
    d = [d, di];
end

D = ones(N, 3);
for i = 1:N
    h = 1 - ((d(i) - min(d))./(max(d) - min(d)));
    r = max_color(1) + (min_color(1)-max_color(1)) * h;
    g = max_color(2) + (min_color(2)-max_color(2)) * h;
    b = max_color(3) + (min_color(3)-max_color(3)) * h;
    D(i,:) = [r,g,b];
end

figure;
axis equal
scatter3(X,Y,Z,S,D);
xlabel('mm');
ylabel('mm');
zlabel('mm');
% set(gca, 'XLim',[400 600]); 
% set(gca, 'ZLim',[70 90]); 
% set(gca, 'YLim',[300 400]); 
hold on;
grid on;

% figure;
% plot3(X,Y,Z);
% set(gca, 'ZLim',[75 85]); 
% set(gca, 'YLim',[515 525]); 
% xlabel('mm');
% ylabel('mm');
% zlabel('mm');
% hold on;
% grid on;


VX = (Q4' - Q3')/norm(Q4'-Q3');
XYZ = [X,Y,Z]'; % Column vectors
[~,maxD] = max(d);

VY = cross((XYZ(:,maxD) - Q3'),VX);VY = VY/norm(VY);
VZ = cross(VX,VY);VZ = VZ/norm(VZ);

mXYZ = XYZ; R = [VX,VY,VZ];
for i = 1:N
    mXYZ(:,i) = R\(XYZ(:,i) - Q3');
end

figure;
axis equal
scatter(mXYZ(2,:),mXYZ(3,:),S,D,'filled');
xlabel('mm');
ylabel('mm');
zlabel('mm');
hold on;
grid on;

ret = d;


end



