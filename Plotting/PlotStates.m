function PlotStates(Robot, states)
% Figure setup
StatePlot = figure;
set(StatePlot,'Name','MIT Balancing Robot States');
FS = 16; title('Balancing Robot States','FontSize',FS);
xlabel('Time, t [s]','FontSize',FS);
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
if (states(1)), plot(x,y(:,1),'b'); end
if (states(5)), plot(x,y(:,5),'b--'); end

% dthw
if (states(2)), plot(x,y(:,2),'r'); end
if (states(6)), plot(x,y(:,6),'r--'); end

% thp
if (states(3)), plot(x,y(:,3),'y'); end
if (states(7)), plot(x,y(:,7),'y--'); end

% dthp
if (states(4)), plot(x,y(:,4),'g'); end
if (states(8)), plot(x,y(:,8),'g--'); end

% Tau
if (states(9)), plot(x,y(:,9)./x,'black'); end