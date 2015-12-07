%%
function dx = dynamics(t,x,Robot,xd)
fprintf('%f\n',t);

% Add current state to the Robot
Robot.States = x(1:4,1);

% System model parameters
g = 9.81;
mp = Robot.Parameters.model.mp;
L = Robot.Parameters.model.L;
Ip = Robot.Parameters.model.Ip;

mw = Robot.Parameters.model.mw;
r = Robot.Parameters.model.r;
Iw = Robot.Parameters.model.Iw;

% Desired state trajectory
xd = eval(xd);

% Linearized system response: dx = (A-B*K)*(x-xd) = Acl*xE
dxLin = (Robot.Controller.LQR.A - ...
    Robot.Controller.LQR.B*Robot.Controller.LQR.K)*(x(5:8,1) - xd);

% Controller input: u = -K*(x-xd)
mp = Robot.Parameters.actual.mp;
L = Robot.Parameters.actual.L;
Ip = Robot.Parameters.actual.Ip;

Robot.Controller.Adaptive.K = x(10:13)';
Tau = -Robot.Controller.Adaptive.K*(x(1:4,1) - xd);
%Tau = -Robot.Controller.LQR.K*(x(1:4,1) - xd);

% Nonlinear system response: dx = f(x,u)
thw = x(1); dthw = x(2); thp = x(3); dthp = x(4);
dxNonlin = eval(Robot.Dynamics.Nonlinear);

% Simulation output
dKadaptive = AdaptiveController(Robot, x(5:8));
dx = [dxNonlin;dxLin;Tau;dKadaptive];
end