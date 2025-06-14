This repository includes the files for controlling four servo motors using hand gestures. Basically, each motor is controlled by opening and closing one of the fingers on the right hand (thumb, index, middle and ring). For the purposes of this demo, motors were connected to four fingers of a mechanical metamaterial-based gripper mechanism. 

How to use the files in this repo:
* ArduinoGestureSerial.ino -> Arduino code to be deployed. Communicates through serial and moves motors towards the desired positions. 
* GripperControlGUI.py -> Python code that communicates with Arduino through serial. Includes 5 modes (two for hand gesture driven modes, three for controlling motors with preprogrammed movements)
* GripperDemoWiring.png -> Shows the wiring diagram for the motors and Arduino
* STLs for printing the rigid and flexible sections of the gripper. 

Please follow the links below for more information. 

* Video of the demo (coming soon!)
* [Publication related to this demo](https://dl.acm.org/doi/10.1145/3689050.3704942)
* [Github repository for design and simulation of presented shape-changing interfaces](https://github.com/aotaran/MetamaterialDesignAndSimulation)

If you have any questions, please email ata.otaran@gmail.com.
