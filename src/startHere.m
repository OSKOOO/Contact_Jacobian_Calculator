function [robot, Contact_Jacobian] = startHere()
    clc; clear; close all;
    global contactFrame;
   
    % Prompt for URDF
    disp('Please enter the path to the URDF file.');
    disp(' e.g., models/cassie.urdf');
    URDF = input('URDF path: ', 's'); 
    disp('**************************************************************')
    disp(' ')

    if exist(URDF, 'file') ~= 2
        disp('Error: URDF file not found.');
        return;
    end

    disp('Please enter the offset frame from the last joint to the contact point.');

    % Prompt for the offset vector
    disp('Enter the offset frame as [x; y; z] in meters:');
    contactFrame = input('Offset vector (e.g., [0; 0.0; -0.04]): ');

    % Validate the input
    while length(contactFrame) ~= 3 || ~isnumeric(contactFrame)
        disp('Invalid input. Please enter a 3-element numeric vector.');
        contactFrame = input('Offset frame (e.g., [0; 0.0; -0.04]): ');
    end

    % Append the homogeneous coordinate
    contactFrame = [contactFrame; 1];

    % Parse the URDF file to get robot configuration
    [ robot, Contact_Jacobian, Rotm_foot] = parseURDF(URDF);

    % Save Contact_Jacobian to the MATLAB base workspace
    assignin('base', 'Contact_Jacobian', Contact_Jacobian);  
    disp('************************************************************************************')
    disp('Contact jacobian analytical function is saved to the workspace and ready to be used.')
end
