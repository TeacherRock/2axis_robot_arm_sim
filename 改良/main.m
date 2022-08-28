clc,clear,close all;
sampT = 0.001;
%      t    px1     vx1    ax1    px2    vx2   ax2
CP = [ 0,   -0.05,  0,     0,     0.1,   0,    0  ; ...
       5,   0,      0.25,  0.25,  0.12,  0,    0  ; ...
       10,  0.05,   0,     0,     0.15,  0,    0  ];

type1 = 'Linear';
type2 = 'CTC';
type3 = 'DFF';
robot = Robot();
controller1 = Controller(type1);
controller2 = Controller(type2);
controller3 = Controller(type3);

Output1 = robot.PosError();
Output2 = robot.PosError();
Output3 = robot.PosError();
t = sampT : sampT : length(Output1)*sampT;
plot(t, Output1, t, Output2, t, Output3);
legend('PD' ,'CTC' ,'DFF');


