function PlotStates(Robot, states)
% Figure setup
StatePlot = figure;
set(StatePlot,'Name','MIT Balancing Robot States');
FS = 16; title('Balancing Robot States','FontSize',FS);
set(0, 'currentfigure', StatePlot); set(gcf,'Color','w'); hold on;

% Add a default for plotted states
if (nargin < 2), states = ones(9,1); end

% Pull out trajectory
x = Robot.traj.x;
y = Robot.traj.y;

% Kinematic system parameters
r = Robot.Parameters.actual.r;
L = Robot.Parameters.actual.L;

% thw
subplot(2,2,1); hold on; grid on;
ylabel('Displacement, x [m]','FontSize',12*FS/16);
xlabel('Time, t [s]','FontSize',12*FS/16);
if (states(1)), plot(x,r*y(:,1),'b'); end
if (states(5)), plot(x,r*y(:,5),'b--'); end
plot(x,0*y(:,9),'black--');

% thp
subplot(2,2,2); hold on; grid on;
ylabel('Angle, th [deg]','FontSize',12*FS/16);
xlabel('Time, t [s]','FontSize',12*FS/16);
if (states(3)), plot(x,180/pi*y(:,3),'r'); end
if (states(7)), plot(x,180/pi*y(:,7),'r--'); end
plot(x,0*y(:,9),'black--');

% dthw
subplot(2,2,3); hold on; grid on;
ylabel('Velocity, dx [m/s]','FontSize',12*FS/16);
xlabel('Time, t [s]','FontSize',12*FS/16);
if (states(2)), plot(x,r*y(:,2),'b'); end
if (states(6)), plot(x,r*y(:,6),'b--'); end
plot(x,0*y(:,9),'black--');

% dthp
subplot(2,2,4); hold on; grid on;
ylabel('Angular Velocity, dth [rad/s]','FontSize',12*FS/16);
xlabel('Time, t [s]','FontSize',12*FS/16);
if (states(4)), plot(x,y(:,4),'r'); end
if (states(8)), plot(x,y(:,8),'r--'); end
plot(x,0*y(:,9),'black--');

% Tau
%if (states(9)), plot(x,y(:,9)./x,'black'); end

figure; hold on
plot(x,y(:,10),'black--');
plot(x,y(:,11),'black--');
plot(x,y(:,12),'black--');
plot(x,y(:,13),'black--');