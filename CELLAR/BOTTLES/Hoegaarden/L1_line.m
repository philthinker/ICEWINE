clc;
clear;

% to change the color of error graph
min_color = [100, 149, 237];
max_color = [0, 0, 128];

data = xlsread('L1_200mms1.xls');
len = length(data);
X1 = data(8:len, 3);
Y1 = data(8:len, 4);
Z1 = data(8:len, 5);

x_start = mean(X1(1:80));
x_end = mean(X1((length(X1)-100):length(X1)));
y_start = mean(Y1(1:80));
y_end = mean(Y1((length(Y1)-100):length(Y1)));
z_start = mean(Z1(1:80));
z_end = mean(Z1((length(Z1)-100):length(Z1)));

X1 = X1(81:len-101);
Y1 = Y1(81:len-101);
Z1 = Z1(81:len-101);
Start = [x_start, y_start, z_start];
End = [x_end, y_end, z_end];
% [d] = fun77(X1, Y1, Z1, Start, End, min_color, max_color);

[d] = DrawLine(X1, Y1, Z1, Start, End, min_color, max_color);

