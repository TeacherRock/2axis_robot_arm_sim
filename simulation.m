clc, clear, close all;

command_1 =  load("axis1.txt");
command_2 =  load("axis2.txt");
xy = load("xy.txt");
ts = 0.001;
tf = 10;
T = ts : ts : tf-ts;

%% J, Dm, fm

l1 = 0.24;
l2 = 0.24;
c1 = 0.12;
c2 = 0.12;
m1 = 2;
m2 = 2;
I1 = 0.0267;
I2 = 0.0267;

%%
% Simulation
Output1 = Simulation(command_1, command_2, T, 1);
Output2 = Simulation(command_1, command_2, T, 2);
Output3 = Simulation(command_1, command_2, T, 3);

% Compare
%name = fieldnames(Output1);

% plot(T(2:9999), Output1.(name{1})(:,2:9999));

%%
[x , y, E] = deal(zeros(length(Output1.P),3));
i = 1;
for t = ts : ts : tf-ts
    x(i,1) = l1*cos(Output1.P(1,i)) + l2*cos(Output1.P(1,i) + Output1.P(2,i)); 
    y(i,1) = l1*sin(Output1.P(1,i)) + l2*sin(Output1.P(1,i) + Output1.P(2,i));
    E(i,1) = sqrt((x(i,1) - xy(i,1))^2 + (y(i,1) - xy(i,2) )^2);
    
    x(i,2) = l1*cos(Output2.P(1,i)) + l2*cos(Output2.P(1,i) + Output2.P(2,i)); 
    y(i,2) = l1*sin(Output2.P(1,i)) + l2*sin(Output2.P(1,i) + Output2.P(2,i));
    E(i,2) = sqrt((x(i,2) - xy(i,1))^2 + (y(i,2) - xy(i,2) )^2);
    
    x(i,3) = l1*cos(Output3.P(1,i)) + l2*cos(Output3.P(1,i) + Output3.P(2,i)); 
    y(i,3) = l1*sin(Output3.P(1,i)) + l2*sin(Output3.P(1,i) + Output3.P(2,i));
    E(i,3) = sqrt((x(i,3) - xy(i,1))^2 + (y(i,3) - xy(i,2) )^2);
    i = i+1;
end

plot(T(2:9999), E(2:9999,1));
% plot(T(2:9999), E(2:9999,2));
% plot(T(2:9999), E(2:9999,1), T(2:9999), E(2:9999,2));
% plot(T(2:9999), E(2:9999,3));
% plot(T(2:9999), E(2:9999,1), T(2:9999), E(2:9999,2), T(2:9999), E(2:9999,3));
% legend('PD' ,'CTC' ,'DFF');

%% Function
function Output = Simulation(Command_1, Command_2, T, Controller_type)

ts = 0.001;
ax = 2;
l1 = 0.24;    l2 = 0.24;
c1 = 0.12;    c2 = 0.12;
m1 = 2;       m2 = 2;
I1 = 0.0267;  I2 = 0.0267;

P = [Command_1(1, 1); Command_2(1, 1)];
[V, A] = deal(zeros(ax,1));

[PCmd, VCmd, ACmd] = deal(zeros(ax, length(Command_1(:,1))));
PCmd(1,:) = Command_1(:, 1)';
VCmd(1,:) = Command_1(:, 2)';
ACmd(1,:) = Command_1(:, 3)';
PCmd(2,:) = Command_2(:, 1)';
VCmd(2,:) = Command_2(:, 2)';
ACmd(2,:) = Command_2(:, 3)';

nc = length(T);

[rec.P, rec.V, rec.A, rec.T, rec.F] = deal(zeros(ax, nc));

for i = 2 : nc
    % M ,C
    [M ,C] = deal(zeros(2, 2));
    
    M(1,1) = I1 + I2 + m1*c1^2 + m2*(l1^2 + c2^2 + 2*l1*c2*cos(P(2,:)));
    M(1,2) = I2 + m2*c2^2 + m2*l1*c2*cos(P(2,1));
    M(2,1) = M(1,2);
    M(2,2) = I2 + m2*c2^2;
    C(1,1) = -1*m2*l1*c2*V(2,1)*sin(P(2,1));
    C(1,2) = -1*m2*l1*c2*(V(1,1) + V(2,1))*sin(P(2,1));
    C(2,1) = m2*l1*c2*V(1,1)*sin(P(2,1));
    C(2,2) = 0;
    
    if Controller_type == 1
        %PD
        Kp = [500;500];     Kv = [0; 0.1];
    elseif Controller_type == 2
        %CTC
        Kp = [1000;1000];     Kv = [58.768434; 300];
    elseif  Controller_type == 3 
        %DFF
        Kp = 1268;     Kv = 0.01;
    end
    
    % Controller
    if Controller_type == 1
        %PD
        ControllerTorque = Kv .* (VCmd(:, i-1) - V(:,1)) + Kp .* (PCmd(:, i-1) - P(:,1));
    elseif Controller_type == 2
        %CTC
        ControllerTorque = M*(Kv .* (VCmd(:, i-1) - V(:,1)) + Kp .* (PCmd(:, i-1) - P(:,1)) + ACmd(:, i-1) ) + C*VCmd(:,i-1);
    elseif  Controller_type == 3 
        %DFF
        ControllerTorque = M*ACmd(:, i-1)  + C*VCmd(:,i-1) + Kv .* (VCmd(:, i-1) - V(:,1)) + Kp .* (PCmd(:, i-1) - P(:,1));
    end
    
    % Calculate Friction
    F = 0;
    
    % Calculate A || DDM Diret Dynamic Model
    A = M\(ControllerTorque - C * V );
   
    % Integral V, P
    V = V + A * ts;
    P = P + V * ts;
    
    
    % record
    rec.P(:, i) = P;
    rec.V(:, i) = V;
    rec.A(:, i) = A;
    rec.T(:, i) = ControllerTorque;
    rec.F(:, i) = F;

end
Output = rec;
end
