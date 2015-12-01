function BalancingRobot = CreateBalancingRobot

BRStruct = struct();
BRStruct.Name = 'Balancing Robot';

%% Symbolic dynamics of the robot
% Nonlinear dynamics
syms g mp Ip mw Iw L r thp dthp ddthp thw dthw ddthw Tau real
BRStruct.Dynamics.Nonlinear = [dthw;...
    ((Ip + mp*L^2 - mp*r*L*cos(thp))*Tau - (Ip + mp*L^2)*mp*r*L*sin(thp)*dthp^2 + mp^2*g*r*L^2*cos(thp)*sin(thp))/((Ip + mp*L^2)*(Iw + mp*r^2 + mw*r^2) - (mp*r*L*cos(thp))^2);...
    dthp;...
    (-(Iw + (mw + mp)*r^2 + mp*r*L*cos(thp))*Tau - (mp*r*L)^2*cos(thp)*sin(thp)*dthp^2 + (Iw + (mw + mp)*r^2)*mp*g*L*sin(thp))/((Ip + mp*L^2)*(Iw + mp*r^2 + mw*r^2) - (mp*r*L*cos(thp))^2)];

% Linearized State-Space
BRStruct.Dynamics.Linear.A = [0, 1, 0, 0;...
    0, 0, (mp^2*g*r*L^2)/((Ip + mp*L^2)*(Iw + mp*r^2 + mw*r^2) - (mp*r*L)^2), 0;...
    0, 0, 0, 1;...
    0, 0, ((Iw + (mw + mp)*r^2)*mp*g*L)/((Ip + mp*L^2)*(Iw + mp*r^2 + mw*r^2) - (mp*r*L)^2), 0];
BRStruct.Dynamics.Linear.B = [0;...
    (Ip + mp*L*(L - r))/((Ip + mp*L^2)*(Iw + mp*r^2 + mw*r^2) - (mp*r*L)^2);...
    0;...
    -(Iw + (mw + mp)*r^2 + mp*r*L)/((Ip + mp*L^2)*(Iw + mp*r^2 + mw*r^2) - (mp*r*L)^2)];
BRStruct.Dynamics.Linear.C = [1, 0, 0, 0;...
    0, 0, 1, 0];
BRStruct.Dynamics.Linear.D = [0;...
    0];

%% Robot Model Parameters
BRStruct.Parameters.model.mp = 513.3e-3;
BRStruct.Parameters.model.L = 80e-3;
BRStruct.Parameters.model.Ip = BRStruct.Parameters.model.mp*BRStruct.Parameters.model.L^2;

BRStruct.Parameters.model.mw = 7.2e-3;
BRStruct.Parameters.model.r = 16e-3;
BRStruct.Parameters.model.Iw = BRStruct.Parameters.model.mw*BRStruct.Parameters.model.r^2/2;

%% Robot Actual Parameters
BRStruct.Parameters.actual.mp = 1603.3e-3;
BRStruct.Parameters.actual.L = 85e-3;
BRStruct.Parameters.actual.Ip = BRStruct.Parameters.actual.mp*BRStruct.Parameters.actual.L^2;

BRStruct.Parameters.actual.mw = 7.2e-3;
BRStruct.Parameters.actual.r = 16e-3;
BRStruct.Parameters.actual.Iw = BRStruct.Parameters.actual.mw*BRStruct.Parameters.actual.r^2/2;

%% Robot States
BRStruct.States = [0;...    % thw  (wheel angle)
    0;...                   % dthw (wheel angular velocity)
    0;...                   % thp  (pendulum angle)
    0];                     % dthp (pendulum angular velocity)

%% LQR Controller
[K, A, B] = LQRController(BRStruct);
BRStruct.Controller.LQR.K = K;
BRStruct.Controller.LQR.A = A;
BRStruct.Controller.LQR.B = B;

BalancingRobot = Robot(BRStruct);
end

