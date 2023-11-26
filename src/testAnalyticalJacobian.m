% Analytical Jacobian Test Script
%
% This MATLAB script is designed to test the analytical Jacobian calculator for a bipedal robotic system.
% It allows users to input their own joint angles and Euler angles, calculates the Jacobian matrix
% using these inputs, and then compares the result with an expected Jacobian matrix provided by the user.
% The script provides feedback on whether the calculated Jacobian matches the expected one.
%
% Usage:
% 1. Run the script in MATLAB.
% 2. When prompted, enter the joint angles as a vector (size depends on the robot configuration).
% 3. Enter the Euler angles for XYZ rotation as a 3-element vector.
% 4. Enter the expected Jacobian matrix (size should be 12x<number_of_joints>).
% 5. The script will display whether the calculated Jacobian matches the expected Jacobian.
%
% Author: Omar Kolt
% Date: Nov 21, 2023
% License: MIT License
% Note: This script is part of an educational project. It is intended for learning and demonstration purposes.
% All rights reserved.
% Please refer to README.md 

clear; clc; close all;

% Parse the URDF file to get robot configuration
[robot, Contact_Jacobian] = startHere();

% Determine the number of joints from the robot structure
numJoints = length(robot.Joints);

% Inform the user of the required input size for joint angles
disp(['Enter joint angles (in radians) as a ', num2str(numJoints), '-element vector:']);
joint_angles = input('Joint angles: ');

% Check if the user input has the correct size for joint angles
while length(joint_angles) ~= numJoints
    disp(['Incorrect number of elements. Please enter ', num2str(numJoints), ' joint angles:']);
    joint_angles = input('Joint angles: ');
end

% Prompt for Euler angles
disp('Enter Euler angles (in radians) for XYZ rotation as a 3-element vector:');
euler_angles = input('Euler angles [alpha, beta, gamma]: ');

% Convert Euler angles to rotation matrix
RR = reshape(eul2rotm(euler_angles, 'XYZ'), 9, 1);

% Determine the size of the Jacobian matrix
expected_Jacobian_rows = 12;
disp(['Enter the expected Jacobian matrix (', num2str(expected_Jacobian_rows), 'x', num2str(numJoints), ' matrix):']);
expected_Jacobian = input('Expected Jacobian: ');

% Check if the user input has the correct size for the Jacobian matrix
[rows, cols] = size(expected_Jacobian);
while rows ~= expected_Jacobian_rows || cols ~= numJoints
    disp(['Incorrect size of the matrix. Please enter a ', num2str(expected_Jacobian_rows), 'x', num2str(numJoints), ' matrix:']);
    expected_Jacobian = input('Expected Jacobian: ');
    [rows, cols] = size(expected_Jacobian);
end

% Calculate Jacobian based on user input
Jc = Contact_Jacobian(joint_angles(1), joint_angles(2), joint_angles(3), joint_angles(4), joint_angles(5), joint_angles(6), joint_angles(7), joint_angles(8), joint_angles(9), joint_angles(10), RR(1), RR(2), RR(3), RR(4), RR(5), RR(6), RR(7), RR(8), RR(9));

% Comparison
if isequal(Jc, expected_Jacobian)
    disp('***************************************************************');
    disp('***************************************************************');    
    disp('Success! The calculated Jacobian matches the expected Jacobian.');
else
    disp('***************************************************************');
    disp('***************************************************************'); 
    disp('Calculated Jacobian:');
    disp(Jc);
    disp('Expected Jacobian:');
    disp(expected_Jacobian);
    
    if(size(Jc) == size(expected_Jacobian))
        disp('Mismatched elements:');
        disp(Jc==expected_Jacobian);    
        disp('Mismatch detected. The calculated Jacobian does not match the expected Jacobian.');
    else
        disp('Mismatch detected. The calculated Jacobian does not match the expected Jacobian.');
    end

end

