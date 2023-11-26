function [Contact_Jacobian, Rotm_foot] = Formulate_Contact_Jacobian(robot)
    global contactFrame;
    % Number of revolute joints
    numRevoluteJoints = length(robot.Joints);

    % Adjust the symbolic variable 'q' based on the number of revolute joints
    q = sym('q', [numRevoluteJoints, 1]);
    
    % Split 'q' into right and left leg joint variables
    % Assuming the first half of the joints belong to the right leg and the second half to the left
    % Adjust this logic based on your robot's configuration
    halfNumJoints = ceil(numRevoluteJoints / 2);
    q_R = q(1:halfNumJoints, 1);
    q_L = q(halfNumJoints+1:end, 1);

    R = sym('r', [9, 1]); % input R as vector
    R = [reshape(R, [3, 3]), zeros(3, 1); zeros(1, 3), 1];

    % Extract link positions from the robot structure (obtained from parseURDF)
    link = repmat({zeros(3, 1)}, halfNumJoints, 1);
    for i = 1:min(length(robot.Joints), halfNumJoints)
        link{i} = robot.Joints(i).Origin';
    end

% Distance between conact wrench and the cinal rev joint get foot(1:3) from user   
% foot = [0; 0.001; -0.04; 1];

% Translation matrices
T_R=repmat({eye(4)},halfNumJoints,1);
T_L=repmat({eye(4)},halfNumJoints,1);
for i=1:halfNumJoints
    T_R{i}(1:3,4)=link{i};
    T_L{i}(1:3,4)=[1;-1;1].*link{i};
end


for i = 1:min(length(robot.Joints), halfNumJoints)
    axis = robot.Joints(i).Axis;
    if isequal(axis, [0, 0, 1]) || isequal(axis, [0, 0, -1]) % Yaw
        R_R{i} = [cos(q_R(i)),-sin(q_R(i)),0,0; sin(q_R(i)),cos(q_R(i)),0,0; 0,0,1,0; 0,0,0,1];
        R_L{i} = [cos(q_L(i)),-sin(q_L(i)),0,0; sin(q_L(i)),cos(q_L(i)),0,0; 0,0,1,0; 0,0,0,1];
    elseif isequal(axis, [1, 0, 0]) || isequal(axis, [-1, 0, 0]) % Roll
        R_R{i}=[1,0,0,0;0,cos(q_R(i)),-sin(q_R(i)),0;0,sin(q_R(i)),cos(q_R(i)),0;0,0,0,1];
        R_L{i}=[1,0,0,0;0,cos(q_L(i)),-sin(q_L(i)),0;0,sin(q_L(i)),cos(q_L(i)),0;0,0,0,1];
    elseif isequal(axis, [0, 1, 0]) || isequal(axis, [0, -1, 0]) % Pitch
        R_R{i}=[cos(q_R(i)),0,sin(q_R(i)),0;0,1,0,0;-sin(q_R(i)),0,cos(q_R(i)),0;0,0,0,1];
        R_L{i}=[cos(q_L(i)),0,sin(q_L(i)),0;0,1,0,0;-sin(q_L(i)),0,cos(q_L(i)),0;0,0,0,1];
    else
        disp("ERROR: Invalid axis of rotation");
    end
end

r_R=contactFrame;
r_L=contactFrame;
o_R{1}=R(1:3,1:3);
o_L{1}=R(1:3,1:3);
for i=halfNumJoints:-1:1
    r_R=T_R{i}*R_R{i}*r_R;
    r_L=T_L{i}*R_L{i}*r_L;
    o_R{halfNumJoints+2-i}=o_R{halfNumJoints+1-i}*R_R{halfNumJoints+1-i}(1:3,1:3);
    o_L{halfNumJoints+2-i}=o_L{halfNumJoints+1-i}*R_L{halfNumJoints+1-i}(1:3,1:3);
end
r_R=R*r_R;
r_L=R*r_L;

% Jc=[dr;do]/dq
Jr_R=jacobian(r_R(1:3,1),q_R);
Jr_L=jacobian(r_L(1:3,1),q_L);


% Initialize Jo_R and Jo_L
Jo_R = sym(zeros(3, halfNumJoints));
Jo_L = sym(zeros(3, halfNumJoints));

% Dynamically create Jo_R and Jo_L based on joint axis
for i = 1:halfNumJoints
    axis = robot.Joints(i).Axis;
    
    Jo_R(:, i) = o_R{i} * axis';
    Jo_L(:, i) = o_L{i} * axis';
end

Jc=[Jr_R;Jr_L;Jo_R;Jo_L]; % right force, left force, right moment, left moment 
contact_mapping=[blkdiag(Jc(1:3,:)',Jc(4:6,:)'),blkdiag(Jc(7:9,:)',Jc(10:12,:)')];
Contact_Jacobian=matlabFunction(contact_mapping); % size=[12,5]
% Rotm_foot=matlabFunction([o_R{6};o_L{6}]); % additional output, foot orientation, size=[6,3]
Rotm_foot=[];
end