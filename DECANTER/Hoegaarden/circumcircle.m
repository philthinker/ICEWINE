function [ o ] = circumcircle(x1, x2, x3)
%CIRCULCIRCLE 此处显示有关此函数的摘要
%   此处显示详细说明
    p1 = x2 - x1;
    p2 = x3 - x2;
    M = (x1 + x2) / 2;
    N = (x3 + x2) / 2;
    n = cross(p1, p2);
    n = n / norm(n);
    mn = N - M;
    L1 = cross(p1, n);
    L1 = L1 / norm(L1);
    L2 = cross(p2, n);
    L2 = L2 / norm(L2);
    if L1(:,2) * L1(:,1) * L2(:,1) * L2(:,2) == 0 || (L1(:,1)*L2(:,2)-L2(:,1)*L1(:,2)) == 0 || (L2(:,1)*L1(:,2)-L2(:,2)*L1(:,1)) == 0
        o = NaN
    else
        a = (L2(:,2)*mn(:,1)-L2(:,1)*mn(:,2))/(L1(:,1)*L2(:,2)-L2(:,1)*L1(:,2));
        b = (L1(:,2)*mn(:,1)-mn(:,2)*L1(:,1))/(L2(:,1)*L1(:,2)-L2(:,2)*L1(:,1));
        o = (M + a * L1);
    end
end

