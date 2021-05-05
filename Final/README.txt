Final: This part of the repository contains all the code relating to the Final Demo.

Demo2Workspace.ino                -   Contains the code from Demo2 that was altered for this demo. This was not used in the final implementation;
                                      however, it provide a good backbone for FinalDemoNoFSM.ino
FinalDemoNoFSM.ino                -   Contains all the code used on the Arduino for the control/localization subsystems. This system does not have 
                                      its own finite-state-machine in it and instead, it only takes commands from the raspberry pi.
ArucoDetectionSchemeAndFSM.py     -   Contains the robot finite state machine, marker detection, and system integration.
