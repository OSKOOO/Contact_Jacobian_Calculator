function robot = parseURDF

urdfFile = 'robot.urdf';
VisualizeKinematicTree (urdfFile);

% Read the URDF file
xmlData = xmlread(urdfFile);

% Initialize structure to hold robot data
robot = struct();
robot.Joints = [];

% Extract all joint elements
joints = xmlData.getElementsByTagName('joint');
numJoints = joints.getLength;

% Initialize link array for the right leg
link = repmat({zeros(3, 1)}, 5, 1);

% Counter for right leg joints
rightLegJointCount = 0;

% Loop through all joints
for i = 0:numJoints-1
    joint = joints.item(i);
    jointName   = char(joint.getAttribute('name'));
    jointType   = char(joint.getAttribute('type'));

    % Process only revolute joints
    if strcmp(jointType, 'revolute')
        % Get the origin attribute
        jointOrigin = joint.getElementsByTagName('origin');
        jointOriginXYZ = char(jointOrigin.item(0).getAttribute('xyz'));
        jointOriginXYZ = str2num(jointOriginXYZ); % Convert string to numeric array

        % Store joint data in robot structure
        robot.Joints = [robot.Joints; struct('Name', jointName, 'Type', jointType, 'Origin', jointOriginXYZ)];

        % Check if the joint belongs to the right leg (based on jointName or other criteria)
        if isRightLegJoint(jointName)
            rightLegJointCount = rightLegJointCount + 1;

            % Populate the link array with joint origin
            if rightLegJointCount <= length(link)
                link{rightLegJointCount} = jointOriginXYZ';
            end
        end
    end
end

% Assuming isRightLegJoint is a function that determines if a joint is part of the right leg
function isRight = isRightLegJoint(jointName)
    % Implement logic to determine if the joint is part of the right leg
    isRight = contains(jointName, 'R_'); % Example criterion
end

% Display link information
disp('Right leg link origins:');
for i = 1:length(link)
    disp(['Link ', num2str(i), ': ', mat2str(link{i})]);
end

end
