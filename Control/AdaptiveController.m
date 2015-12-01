function dKadaptive = AdaptiveController(Robot, xm)
% System physical parameters
g = 9.81;

% Actual Pendulum Parameters
mp = Robot.Parameters.actual.mp;
L = Robot.Parameters.actual.L;
Ip = Robot.Parameters.actual.Ip;

% Actual Wheel Parameters
mw = Robot.Parameters.actual.mw;
r = Robot.Parameters.actual.r;
Iw = Robot.Parameters.actual.Iw; 

Am = (Robot.Controller.LQR.A - Robot.Controller.LQR.B*Robot.Controller.LQR.K);

A = eval(Robot.Dynamics.Linear.A);
B = eval(Robot.Dynamics.Linear.B);

e = Robot.States - xm;

Q = eye(4);
P = care(Am,B,Q);%zeros(size(Am,1),1), Q);

Psi = -B*xm';%Robot.States';
Eta = 0.1*eye(4);Eta(2,2) = 0.1;Eta(4,4) = 0.1;
dKadaptive = -Eta*Psi'*P*e;

