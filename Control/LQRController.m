function [K, A, B, S] = LQRController(Robot)
% System model parameters
g = 9.81;
mp = Robot.Parameters.model.mp;
L = Robot.Parameters.model.L;
Ip = Robot.Parameters.model.Ip;
mw = Robot.Parameters.model.mw;
r = Robot.Parameters.model.r;
Iw = Robot.Parameters.model.Iw;

% Linearized System: dx = A*x + B*u
A = eval(Robot.Dynamics.Linear.A);
B = eval(Robot.Dynamics.Linear.B);

% LQR Control Design
Q = eye(4); Q(2,2) = 1;
R = 0.1;
[K, S] = lqr(A, B, Q, R);