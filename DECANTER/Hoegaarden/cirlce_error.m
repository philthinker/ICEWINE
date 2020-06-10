clc
clear;
data = xlsread('D:\动捕测试数据\NOKOV动捕测试数据\圆弧试验\数据\v100-8\v100d100-81');
len = length(data);
X1 = data(8:len, 3);
Y1 = data(8:len, 4);
Z1 = data(8:len, 5);
len = length(X1);

avg_center = [nanmean(X1), nanmean(Y1), nanmean(Z1)]
figure(1)

plot3(X1, Y1, Z1, 'r.', 'MarkerSize', 5)
axis equal
grid on
hold on
xlabel('mm');
ylabel('mm');
zlabel('mm');
scatter3(avg_center(1), avg_center(2), avg_center(3), 'r','filled')
axis equal
hold on
grid on

step = 10;
d_th = 50;
o = [];
% get cirlce center
for i = 1:step:len
    p1 = [X1(i), Y1(i), Z1(i)];
    for j = i+1:step:len
        p2 = [X1(j), Y1(j), Z1(j)];
        d12 = norm(p1 - p2);
        if d12 > d_th
            for k = j+1:step:len
                p3 = [X1(k), Y1(k), Z1(k)];
                d23 = norm(p2 - p3);
                d13 = norm(p1 - p3);
                if d23 > d_th && d13 > d_th && d13 > d12 && d13 > d23
                    o = [o, (circumcircle(p1, p2, p3))'];
                end
            end
        end
    end
end
center = [nanmean(o(1,:)),nanmean(o(2,:)),nanmean(o(3,:))]
scatter3(center(1), center(2), center(3), 'g','filled')
axis equal
hold on
%scatter3(o(1,:), o(2,:), o(3,:), 'b')
%hold on

% get circle radius
radius_v = [];
for i = 1:len
    p = [X1(i), Y1(i), Z1(i)] - center;
    radius_v = [radius_v, norm(p)];
end
radius = nanmean(radius_v)

% get circle plane normal vector
norm_v = [];
for i = 1:step:len
    p1 = [X1(i), Y1(i), Z1(i)];
    for j = i+1:step:len
        p2 = [X1(j), Y1(j), Z1(j)];
        d12 = norm(p1 - p2);
        if d12 > d_th
            tmp = (cross(p1 - center, p2 - center))';
            tmp = tmp / norm(tmp);
            if length(norm_v) > 0
                if dot(norm_v(:,1), tmp) < 0
                    tmp = - tmp;
                end
            end
            norm_v = [norm_v, tmp];
        end
    end
end
normal = [nanmean(norm_v(1,:)),nanmean(norm_v(2,:)),nanmean(norm_v(3,:))];
normal = normal / norm(normal)
tmp = [center', (center + 10 * normal)'];
plot3(tmp(1,:), tmp(2,:), tmp(3,:),'Color',[0.25,0.41,0.88]);% 'og-'
xlabel('mm');
ylabel('mm');
zlabel('mm');
axis equal
hold on

% get error
error_v = [];
circle_p = [];
for i = 1:len
    p = [X1(i), Y1(i), Z1(i)];
    v = cross(normal, p - center);
    u = cross(v, normal);
    u = u / norm(u);
    q = center + u * radius;
    circle_p = [circle_p, q'];
    error_v = [error_v, norm(q - p)];
end
plot3(circle_p(1,:), circle_p(2,:), circle_p(3,:), 'b.-', 'MarkerSize', 5 )
hold on

figure(2)
t = 1:length(error_v);
plot(t, error_v,'Color',[0.25,0.41,0.88])
grid on
xlabel('mm');
ylabel('mm');
zlabel('mm');
max_error = nanmax(error_v)
min_error = nanmin(error_v)
error = nanmean(error_v)

