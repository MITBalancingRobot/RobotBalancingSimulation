%% ================================Robot===================================
% 16.31: Feedback Control Systems
% Final Project: Self-Balancing Robot
% Gerardo Bledt
% November 24, 2015
%
% Simulates the Robot balancing itself using an LQR controller and some
% initial conditions.

classdef Robot
    properties
        Name;
        Dynamics;
        Parameters;
        States;
        Controller;
        traj;
    end
    
    methods
        % Creates the Robot object from the Kinematic Chain struct.
        function Robot = Robot(RobotStruct)
            Robot.Name = (RobotStruct.Name); 
            Robot.Dynamics = (RobotStruct.Dynamics); 
            Robot.Parameters = (RobotStruct.Parameters);
            Robot.States = (RobotStruct.States); 
            Robot.Controller = (RobotStruct.Controller); 
        end
    end
end