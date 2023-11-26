# Analytical Contact Jacobian Calculator for Bipedal Robots using URDF

This project provides a set of MATLAB scripts for visualizing the kinematic tree of a robot, testing the analytical Jacobian, and parsing URDF files to calculate various kinematic properties. The tools are designed to be flexible and adaptable to different robotic configurations.

## Files

- `VisualizeKinematicTree.m`: Generates a visual representation of the robot's kinematic tree.
- `testAnalyticalJacobian.m`: Tests the analytical Jacobian calculator with user-defined inputs.
- `startHere.m`: Main interface for the calculator, guiding the user through inputting necessary parameters.
- `parseURDF.m`: Parses URDF files to extract robot configuration and kinematics.
- `Formulate_Contact_Jacobian.m`: Calculates and constructs the contact Jacobian and returns a Matlab symbolic function
  
## Installation

To use these scripts, clone this repository or download the files into a directory on your machine. Ensure you have MATLAB installed.

```bash
git clone git@github.com:OSKOOO/Contact_Jacobian_Calculator.git
```

## Usage

Run the startHere.m script in MATLAB to begin. This script will guide you through the necessary steps to input data and perform calculations. It interacts with other scripts and utilizes their functionalities to provide results.

## Visualizing Kinematic Tree
```bash
VisualizeKinematicTree('path/to/urdf/file.urdf')
```
<img src="https://github.com/OSKOOO/Contact_Jacobian_Calculator/blob/main/doc/cassie_tree.png" width="500" alt="Kinematic Tree Visualization">

## Project Structure

The project is organized as follows:

    src/: Contains all the source MATLAB scripts.
    src/models/: Place your URDF models here.

It's recommended to keep URDF files in the src/models/ directory and MATLAB scripts in the src/ directory for better organization.

## Contributing
Contributions to this project are welcome. Please fork the repository and submit a pull request with your changes to accommodate your bipedal robot.

## Debugging and future work
  - Make sure you have a naming convention for your joints in the URDF file that follows the examples in /models. E.g., R_knee_joint or right_knee, etc. else you can modify parseURDF.m file to add your naming convention to the list
  - We make a necessary assumption that there is a mirror symmetry between the two legs
  - Working on a jacobian that accounts for floating body rotation
  - Working on adding calculating jacobians between any user-given frames

## License
This project is licensed under the MIT License - see the LICENSE file for details.

## References 
  DRCL-USC,
  "https://github.com/DRCL-USC/Hector_Simulation"




