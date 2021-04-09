# Hyperdrive Test Environment
This robot project is where all new features will be added and tested, to make sure that they can produce predicable values before testing the features on an actual robot.
When the code passes all of its tests in this environment, then a separate robot project in a separate branch will be created to test the code on the robot.
After the robot testing has been completed, then the tested code will be merged with the code from this envionment, where it will wait to be pushed to the main branch.

This environment also provides an example of how to implement Hyperdrive onto your robot. However, the example in the manual will be more accurate.

- The Hyperdrive library is located at [*src/main/java/frc/robot/util/hyperdrive*](https://github.com/BTK203/Hyperdrive/tree/develop/HyperdriveProject/Hyperdrive/src/main/java/frc/robot/util/hyperdrive)
- All basic development and testing will be done in this robot project. This will be to ensure that the library can produce predictable values before pushing the code to a robot for further testing.
- After the code has been tested in this project, a new branch will be created to test the feature on an actual robot.
- After the code has been tested on the robot, it will be re-introduced to this project.
