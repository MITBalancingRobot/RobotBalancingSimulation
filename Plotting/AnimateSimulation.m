function AnimateSimulation(Robot)
% Figure setup
Sim = figure;
set(Sim,'Name','MIT Balancing Robot Simulation');
FS = 16; title('Balancing Robot Simulation','FontSize',FS);
xlabel('X [m]','FontSize',FS); ylabel('Y [m]','FontSize',FS);
pbaspect([40 15 1]); axis([-0.2 0.2 -0.025 0.125]);
set(0, 'currentfigure', Sim); set(gcf,'Color','w'); hold on;
pause(2); drawnow

% Pull out trajectory
x = Robot.traj.x;
y = Robot.traj.y;

% Kinematic system parameters
r = Robot.Parameters.actual.r;
L = Robot.Parameters.actual.L;

% Animate system solution for the time duration
for i = 1:1:length(x)
    % Clear previous timestep
    cla
    
    % Plot the body of the Robot (pendulum)
    plot([y(i,1)*r, y(i,1)*r - L*sin(y(i,3))], [r, r + L*cos(y(i,3))],...
        'Color','r','LineWidth',25);
    
    % Plot the wheel angle state
    plot([y(i,1)*r, y(i,1)*r - r*sin(y(i,1))], [r, r - r*cos(y(i,1))],...
        'Color','black','LineWidth',3);
    
    % Draw the wheel rim
    ang=0:0.01:2*pi;
    xp=r*cos(ang); yp=r*sin(ang);
    plot(y(i,1)*r+xp,r+yp,'LineWidth',3, 'Color','black');
    
    % Draw the ground
    plot([-0.2 0.2],[0,0],'LineWidth',3, 'Color','black');
    drawnow;
end