function VisualizeKinematicTree(urdfFile)
    % Parse the URDF file to get the robot name
    xmlDoc = xmlread(urdfFile);
    robotElement = xmlDoc.getElementsByTagName('robot').item(0);
    robotName = char(robotElement.getAttribute('name'));

    % Check if robot name is empty
    if isempty(robotName)
        disp('Robot name not found in URDF file.');
        return;
    end

    % Get the OS type
    [~, os] = computer;

    % Determine the OS and open the PDF
    if ispc
        disp('Running on Windows');
    elseif isunix
        if ismac
            disp('Running on macOS');
        else
            disp('Running on Linux/Unix');
            % Generate the graphiz file from the URDF file
            [status, cmdout] = system(['urdf_to_graphiz ', urdfFile]);

            % Check if the command was successful
            if status == 0
                % Open the PDF file
                system(['xdg-open ', robotName, '.pdf']);
            else
                disp('Error in executing urdf_to_graphiz command');
                disp('Is ROS installed?');
                disp(cmdout);
            end
        end
    else
        disp('Unknown operating system');
    end
end
