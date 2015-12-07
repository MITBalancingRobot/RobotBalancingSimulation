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
BRStruct.Parameters.model.mp = 0.095 + 0.05;
BRStruct.Parameters.model.L = (0.095*0.015 + 0.05*0.13)/(0.095 + 0.05);
BRStruct.Parameters.model.Ip = BRStruct.Parameters.model.mp*(1.524e-2)^2 + 0.05*0.13^2;

BRStruct.Parameters.model.mw = 7.2e-3;
BRStruct.Parameters.model.r = 4.6990e-2;
BRStruct.Parameters.model.Iw = BRStruct.Parameters.model.mw*BRStruct.Parameters.model.r^2/2;

%% Robot Actual Parameters
mAdd = 0.100;
BRStruct.Parameters.actual.mp = 0.095 + 0.05 + mAdd;
BRStruct.Parameters.actual.L = (0.095*0.015 + (0.05 + mAdd)*0.13)/(0.095 + 0.05 + mAdd);
BRStruct.Parameters.actual.Ip = 95e-3*(1.524e-2)^2 + (0.05 + mAdd)*0.13^2;

BRStruct.Parameters.actual.mw = 7.2e-3;
BRStruct.Parameters.actual.r = 4.6990e-2;
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

