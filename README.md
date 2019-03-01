# SearchAndRescueRobot
High level package that contains ROS packages for our robot bill.
To download and compile, run:

	cd ~/catkin_ws/src
	git clone https://github.com/SeamusJohnston/SearchAndRescueRobot
	cd ..
	catkin_make

Make sure to also download and following installation instructions for WiringPi. 
To download the serial library:

	cd
	git clone https://github.com/wjwwood/serial	
	cd serial
	make
	make install

To run the formatter, make sure you have clang-format-3.9 installed
	
  	sudo apt-get install clang-format-3.9
   
   
Then run clang format on all the files, or just the one you edited:
	
   	cd ~/catkin_ws/src/SearchAndRescueRobot 
    clang-format-3.9 -i -style=file bill_drivers/src/*.cpp
   

## bill_drivers
All firmware and low level control code for writing and reading from actuators and sensors.
To launch all drivers run:
	
    roslaunch bill_drivers drivers.launch
   
To launch a specific driver run:
	
    rosrun bill_drivers driverName
   


## bill_planning
All high level control code for challenge modes, planning, searching and navigation.

## bill_msgs
Contains all custom ROS message definitions.
