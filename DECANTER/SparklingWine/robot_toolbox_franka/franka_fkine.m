% clear;
% clc;
%建立机器人模型
%       theta    d           a        alpha     offset
SL1=Link([0       0.333       0         0        0     ],'modified'); 
SL2=Link([0       0           0         -pi/2    0     ],'modified');
SL3=Link([0       0.316       0         pi/2     0     ],'modified');
SL4=Link([0       0           0.0825    pi/2     0     ],'modified');
SL5=Link([0       0.384       -0.0825   -pi/2    0     ],'modified');
SL6=Link([0       0           0         pi/2     0     ],'modified');
SL7=Link([0       0.2104          0.088     pi/2     0     ],'modified');
modrobot=SerialLink([SL1 SL2 SL3 SL4 SL5 SL6 SL7],'name','franka');
q=[1.4201   -1.5963    0.4092   -1.4924   -1.4205   -0.4113    0.8447];
T1=modrobot.fkine(q)%正解
modrobot.plot(q);
q1=modrobot.ikine(T1)%反解

