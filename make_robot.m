function robot = make_robot()
    %MAKE_ROBOT Creates the kinematic structure of the robot used in the exam
    %   This is a factory function that creates the robot needed for the exam.
    %
    %   Inputs: None
    %   Output: robot - the robot structure, created using Peter Corke's
    %   robotics toolbox
    %
    %   Author: L. Fichera <lfichera@wpi.edu>
    %   Last modified: 4/29/2025
    
    addpath("utils")
    
    L1 = 0.3;
    L2 = 0.3;
    L3 = 0.3;
    
    robot = SerialLink([Revolute('offset', -pi/2, 'a', -L1, 'alpha', 0), ...
                        Revolute('offset', 0, 'a', -L2, 'alpha', 0), ...
                        Revolute('offset', 0, 'a', -L3, 'alpha', 0)], ...
                        'name', 'RRR');
    
    R = [-1 0 0;
         0 0 1;
         0 1 0];
    
    T_aux = make_htm(R, zeros(3,1));
    
    robot.base = T_aux;
    
    end