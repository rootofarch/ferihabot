#!/bin/bash

if [[ `rosversion -d | grep -i kinetic` ]]; then
    if [[ `ls ~/ | grep -i arduino` ]]; then
        sudo apt-get update
        sudo apt-get install -y \
                    arduino \
                    ros-kinetic-rosserial-arduino \
                    ros-kinetic-rosserial
        rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries
        cp arduino/libraries/* ~/Arduino/libraries/
    else
        echo "Please make sure you have Arduino IDE and 'Arduino' folder in home direcetory"
    fi
else
    echo "Please make sure you have ROS Kinetic"
fi
