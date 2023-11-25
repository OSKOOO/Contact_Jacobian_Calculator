function VisualizeKinematicTree = VisualizeKinematicTree(urdfFile)
% Get the OS type
[~, os] = computer;

% Determine the OS
if ispc
    disp('Running on Windows');
elseif isunix
    if ismac
        disp('Running on macOS');
    else
        disp('Running on Linux/Unix');
        % Define the URDF file
        urdfFile = 'models/robot.urdf';

        % Generate the graphiz file from the URDF file
        [status, cmdout] = system(['urdf_to_graphiz ', urdfFile]);

        % Check if the command was successful
        if status == 0
            % Assuming the output file is named 'robot.pdf'
            % FIXE ME: this will not work because the pdf generated is
            % based on the robot name defined in PDF 
            VisualizeKinematicTree = system(['xdg-open my_biped_description.pdf']);
        else
            disp('Error in executing urdf_to_graphiz command');
            disp(cmdout);
        end
    end
else
    disp('Unknown operating system');
end
end