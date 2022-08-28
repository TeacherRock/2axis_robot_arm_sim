%% Function for PSO 
function [Output, Stable] = Pos(Command_1, Command_2, T, Controller_type, K)

ts = 0.001;
ax = 2;
l1 = 0.24;    l2 = 0.24;
c1 = 0.12;    c2 = 0.12;
m1 = 2;       m2 = 2;
I1 = 0.0267;  I2 = 0.0267;

P = [Command_1(1, 1); Command_2(1, 1)];
[V, A, ControllerTorque] = deal(zeros(ax,1));

Kd = reshape(K([1, 2]), [], 1);
Kp = reshape(K([3, 4]), [], 1);

PCmd = [Command_1(:, 1)'; Command_2(:, 1)'];
VCmd = [Command_1(:, 2)'; Command_2(:, 2)'];
ACmd = [Command_1(:, 3)'; Command_2(:, 3)'];

nc = length(T);
% M, C
M_Full = @(P) [I1 + I2 + m1*c1^2 + m2*(l1^2 + c2^2 + 2*l1*c2*cos(P(2))), ...
               I2 + m2*c2^2 + m2*l1*c2*cos(P(2));
               I2 + m2*c2^2 + m2*l1*c2*cos(P(2)), ...
               I2 + m2*c2^2];
           
C_Full = @(P, V) [-1*m2*l1*c2*V(2,1)*sin(P(2,1)), -1*m2*l1*c2*(V(1,1) + V(2,1))*sin(P(2,1));
                   m2*l1*c2*V(1,1)*sin(P(2,1)), 0];

[rec.P, rec.V, rec.A] = deal(zeros(ax, nc));

i = 2; Stable = true;
while (i < nc) 
    % M, C
    M = M_Full(P);
    C = C_Full(P, V);
    
    % Controller
    if Controller_type == 1
        %PD
        ControllerTorque = Kp .* (PCmd(:, i-1) - P) + ...
                           Kd .* (VCmd(:, i-1) - V);
    elseif Controller_type == 2
        %CTC
        ControllerTorque = M * (Kp .* (PCmd(:, i-1) - P) + ...
                                Kd .* (VCmd(:, i-1) - V) + ...
                                ACmd(:, i-1)) + ...
                           C * VCmd(:, i-1);
    elseif  Controller_type == 3 
        %DFF
        ControllerTorque = M * ACmd(:, i-1) + ...
                           C * VCmd(:, i-1) + ...
                           Kp .* (PCmd(:, i-1) - P) + ...
                           Kd .* (VCmd(:, i-1) - V);
    end
    
    % Calculate Friction
    F = 0;
    
    % Calculate A || DDM Diret Dynamic Model
    A = M \ (ControllerTorque - C * V );
   
    % Integral V, P
    V = V + A * ts;
    P = P + V * ts;
    
    
    % record
    rec.P(:, i) = P;
    rec.V(:, i) = V;
    rec.A(:, i) = A;
    
    if sum(isinf(A)) > 0
        Stable = false;
        break
    end
    i = i + 1;

end
Output = rec;
end