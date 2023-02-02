# ros_serial_code
this is a code which can use ros to control device by serial port.
```
mkdir -p ~/catkin_ws/src/
cd ~/catkin_ws/src/
git clone https://github.com/threefruits/ros_serial_code.git
cd ros_serial_code
git submodule update --init --recursive

cd ~/catkin_ws
catkin_make
```