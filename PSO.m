clc, clear, close all;

%bound
LB = [0, 0, 0, 0];
UB = [100, 100, 500, 500];
x_ini = [20, 20, 100, 100];
x_ini_normalize = (x_ini - LB) ./ (UB - LB);


%%
fun = @(x) Optimization(x);

nvars = 4;
lb = zeros(nvars,1);
ub = ones(nvars,1);

options = optimoptions( 'particleswarm', ...
                        'SwarmSize',            50, ...
                        'Display',              'iter', ...
                        'MaxIterations',        100, ...
                        'MaxStallIterations',   10, ...
                        'InitialSwarmMatrix',   x_ini_normalize, ...
                        'FunctionTolerance',    1e-20, ...
                        'UseParallel',          false);

[x, fval, exitflag, output] = particleswarm(fun, nvars, lb, ub, options);
x_re = x .* (UB - LB) + LB;

%%
disp(['Initial: ', num2str(Optimization(x_ini_normalize))])
disp(['Final: ', num2str(Optimization(x))])
disp(['Initial: ', num2str(x_ini)])
disp(['Final: ', num2str(x_re)])

%% function
function [fitness] = Optimization(x)
%還原正歸化
LB = [0, 0, 100, 100];
UB = [1, 1, 400, 400];
x_re = x .* (UB - LB) + LB;

%parameter
ts = 0.001;
tf = 10;
T = ts : ts : tf-ts;
l1 = 0.24;
l2 = 0.24;
c1 = 0.12;
c2 = 0.12;
m1 = 2;
m2 = 2;
I1 = 0.0267;
I2 = 0.0267;

%command
command_1 =  load("axis1.txt");
command_2 =  load("axis2.txt");
xy = load("xy.txt");

[Rec, Stable] = Pos(command_1, command_2, T, 1, x_re);

if Stable
    x = l1.*cos(Rec.P(1,:)) + l2.*cos(Rec.P(1,:) + Rec.P(2,:)); 
    y = l1.*sin(Rec.P(1,:)) + l2.*sin(Rec.P(1,:) + Rec.P(2,:));
    fitness = sum(sqrt((x - xy(:, 1)') .^2 + (y - xy(:, 2)') .^2));
    
else
    fitness = Inf;
end

RatedSpeed = 3000 * 2*pi/60 ;  % 馬達額定轉速 (rad/s) , 單位轉換: (rad/s) = (rpm) * 2*pi/60
GearRatio = 50;  % 各軸減速比
PosBound = [  90  ,  150  ;
             -90  , -150  ] * 0.7 * ( pi / 180 ) ;  % 機構角度限制 (rad)     
VelBound = ( RatedSpeed / GearRatio ) * 0.8 ;  % 機構速度限制 (rad/s)
AccBound = VelBound * 2 ;  % 機構加速度限制: 速度限制*自定倍數 (rad/s^2)

if  any(abs(Rec.P(:, 2)) > PosBound(1, 2)) ...
        || any(abs(Rec.V) > VelBound) || any(abs(Rec.A) > AccBound)
    fitness = Inf;
end

end