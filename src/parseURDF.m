function [robot, Contact_Jacobian, Rotm_foot] = parseURDF(urdfFile)

    VisualizeKinematicTree (urdfFile);

    % Read the URDF file
    xmlData = xmlread(urdfFile);

    % Initialize structure to hold robot data
    robot = struct();
    robot.Joints = [];

    % Temporary arrays for right and left leg joints
    rightLegJoints = [];
    leftLegJoints = [];

    % Extract all joint elements
    joints = xmlData.getElementsByTagName('joint');
    numJoints = joints.getLength;

    % Loop through all joints
    for i = 0:numJoints-1
        joint = joints.item(i);
        jointName = char(joint.getAttribute('name'));
        jointType = char(joint.getAttribute('type'));

        % Process only revolute joints
        if strcmp(jointType, 'revolute') || strcmp(jointType, 'continuous')
            % Get the origin attribute
            jointOrigin = joint.getElementsByTagName('origin');
            jointOriginXYZ = char(jointOrigin.item(0).getAttribute('xyz'));
            jointOriginXYZ = str2num(jointOriginXYZ); % Convert string to numeric array
            
            jointAxis = joint.getElementsByTagName('axis');
            if jointAxis.getLength > 0
                axisXYZ = char(jointAxis.item(0).getAttribute('xyz'));
                axisXYZ = str2num(axisXYZ); % Convert string to numeric array
            else
                axisXYZ = [0, 0, 0]; % Default or error value
            end

            % Store joint data with axis
            jointData = struct('Name', jointName, 'Type', jointType, 'Origin', jointOriginXYZ, 'Axis', axisXYZ);
            if isRightLegJoint(jointName)
                rightLegJoints = [rightLegJoints; jointData];
            else
                leftLegJoints = [leftLegJoints; jointData];
            end
        end
    end

    % Concatenate right and left leg joint data
    robot.Joints = [rightLegJoints; leftLegJoints];

    % Call the function to calculate Contact_Jacobian and Rotm_foot
    [Contact_Jacobian, Rotm_foot] = Formulate_Contact_Jacobian(robot);
end

% Function to determine if a joint is part of the right leg
function isRight = isRightLegJoint(jointName)
    % Check if the joint name contains any of the specified prefixes
    %Modify based on your robot naming convention
    rightLegPrefixes = {'R_', 'right_', 'Right_'};
    isRight = any(contains(rightLegPrefixes, jointName));
end

