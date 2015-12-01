%% ====================Simulate MIT Balancing Robot========================
% 16.31: Feedback Control Systems
% Final Project: Self-Balancing Robot
% Gerardo Bledt
% November 24, 2015
%
% Simulates the Robot balancing itself using an LQR controller and some
% initial conditions.

function BalancingRobot = MITBalancingRobotSimulation
clear; close all; clc;

% Create Balancing Robot
BalancingRobot = CreateBalancingRobot;

% Simulation runtime
tEnd = 10;
dt = 0.01;
tspan = 0:dt:tEnd;

% Initial state vector
thi = -10*pi/180;
x0 = BalancingRobot.States; 
x0(3,1) = thi;
x0 = [x0;x0;0;BalancingRobot.Controller.LQR.K'];

% Desired state trajectories
syms t r real
xd = 0*[-0.1/r;0;0;0];%[-0.2*cos(2*pi*t)/r;0;0;0];

% Simulate the system
opts = odeset('AbsTol',1e-3,'RelTol',1e-3);
[x,y] = ode23t(@dynamics,tspan,x0,opts,BalancingRobot,xd);

% Store Results in the Robot Object
BalancingRobot.Controller.Adaptive.K = y(end,10:13);
BalancingRobot.traj.x = x; BalancingRobot.traj.y = y; 

% Plot the results of the states
PlotStates(BalancingRobot, ones(9,1));

% Animate the system simulation
%AnimateSimulation(BalancingRobot);
end