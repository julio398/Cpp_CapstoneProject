Udacity CppND
Julio Aguilar.  Oct, 10th 2019
Capstone project

// This project is based on the Go Chase it project that I submitted on Feb 28th 2019 as a part of the Robotics Nano degree, however substantial structure and code  have been modified by using the new c++ concepts that I learned on the CppND.

Content of Readme file:

I. Instruccions for building/running the project
II. OS, enviroment information
III. Project description and structure
IV.  Nodes description and classes
V.  About the rubric points and Writeup


I. Instruction for building running the project

Please follow this instructions in order to make a catkin_ws file and copy all the information of the Capstone_project inside the catkin_ws/src directory:

1. cd /home/workspace
2. mkdir catkin_ws
3. cd catkin_ws
4. mkdir src
5. cd ..
6. Copy all contents of Capstone_project into the src folder. 
7. cd catkin_ws
8. catkin_make // Build the project by using catkin_make

After you have succeded building the catkin_ws, please follow this instructions to run the project

1. cd /home/workspace/catkin_ws/src/scripts
2.  ./ball_chaser.sh // Two terminal will open in order to run two launch files:  1. The world.launch file that runs the world file by Gazebo, and the rvizconfig file (rviz)   2. The ball_chaser.launch that run the c++ nodes of ball_chaser. (The program that makes the robot chase a white ball in the Gazebo world).
3. To exit, simply close the Gazebo and rviz windows and press ctrl + c  on each terminal.   Please wait 60 seconds after exit the execution before running the script again. 

II. OS, enviroment information

This project only can run by using Ubuntu/lubuntu 16.04 with ROS Kinetic properly installed. 

To know more details about ROS Kinetic installation please visit: http://wiki.ros.org/kinetic#Installation

Note *We do not promise that this project is going to run properly on newer ROS distributions. 

III. Project description and structure

The purpose of this project is to develop a program that lets a Robot inside a Gazebo world to chase a white ball by using two nodes:  drive_bot and process_image.

On this project we have three (3) packages and one (1) scripts folder, 

Structure:

/src
	/ball_chaser (Package that contains the C++ nodes that our robot needs in order to get motor commands and do image processing)
		/launch
			ball_chaser.launch
		/src
			drive_bot.cpp
			process_image.cpp
		/srv
			DriveToTarget.srv
		CMakeLists.txt
		package.xml
	/my_robot (Package with all Robot and world related files, plugins, etc) 
		launch
			robot_description.launch
			world.launch
		meshes
			hokuyo.dae
		urdf
			my_robot.gazebo
			my_robot.xacro
		worlds
			julio.world
		CMakeLists.txt
		package.xml
	/rvizconfig (This package only contains the rviz config file, strange? No, because the world.launch file launches the rviz config file based on the package name instead that by refering to a address location that would break in a different enviroment)
		my_robot.rviz
		CMakeLists.txt
		package.xml
	/scripts (This folder only contains the script to run the launch files and source the enviroment script: source devel/setup.bash)
		ball_chaser.sh
	CMakeLists.txt
	readme.txt
	
IV. Nodes description and classes


Node: ball_chaser/drive_bot
Class: Handle_Drive
	This node uses a class "Handle_Drive"  that is a call back function that executes each time a drive_bot service is requested, then the requested linear x and angular velocities are publish to the robot wheel joints.

Node: ball_chaser/process_image
Class: PImage_node

	This node uses a class "PImage_node" that contains a call back function that continuously executes and reads the image and loop through each pixel looking for a value equal to 255, if so it drives the motor: forward, left or right. This class also
        includes a drive_robot function that calls the command_robot service to drive the robot in some specied direction. 

* Expected behavior/output after run the program.   The user can place or manipulate the white ball by using gazebo and so far as the robot can see the white ball it will follow it.   You can visualize the robot's camera by using the rviz (Its already configured).

V.	About the rubric points and Writeup

Besides the readme file criteria, this project should satisfy the following criteria from the rest of the rubric:

	- The project demonstrates an understanding of C++ functions and control structures.
		/home/workspace/catkin_ws/src/ball_chaser/src/process_image.cpp line 48
	- The project reads data from a file and process the data, or the program writes data to a file.
		/home/workspace/catkin_ws/src/ball_chaser/src/process_image.cpp line 36
		/home/workspace/catkin_ws/src/ball_chaser/src/drive_bot.cpp line 18
	- The project uses Object Oriented Programming techniques.
		/home/workspace/catkin_ws/src/ball_chaser/src/process_image.cpp line 14
		/home/workspace/catkin_ws/src/ball_chaser/src/drive_bot.cpp line 13
        - Classes use appropriate access specifiers for class members.
		/home/workspace/catkin_ws/src/ball_chaser/src/process_image.cpp line 16
		/home/workspace/catkin_ws/src/ball_chaser/src/drive_bot.cpp line 43
        - Classes encapsulate behavior.
		/home/workspace/catkin_ws/src/ball_chaser/src/process_image.cpp line 20
        - Classes abstract implementation details from their interfaces.
               /home/workspace/catkin_ws/src/ball_chaser/src/process_image.cpp line 19
		/home/workspace/catkin_ws/src/ball_chaser/src/drive_bot.cpp line 30
        - The project makes use of references in function declarations.
		/home/workspace/catkin_ws/src/ball_chaser/src/process_image.cpp line 93
		/home/workspace/catkin_ws/src/ball_chaser/src/drive_bot.cpp line 18

About the write up:  Please see the file named: writeup.pdf on the first level directory of the Capstone_project.





