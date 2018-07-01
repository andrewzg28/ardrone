cmake_minimum_required(VERSION 2.8)
project(ardrone)

find_package(catkin REQUIRED)
catkin_package()

#catkin_python_setup()

install(PROGRAMS src/drone_control.py  
		 src/drone_estado.py  
		 src/drone_video.py  
		 src/joystick_control.py  
		 src/teclado_control.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)