# beacon_combiner_node
Exposes two beacon deployment boards as one 
Must setup udev rules for boards for rosserial
Ex: (serial and bcdDevice attributes will be different, can find what these are with something like udevadm info -a -p  $(udevadm info -q path -n /dev/ttyACM3))
ACTION == "add", SUBSYSTEMS == "usb", ATTRS{manufacturer} == "Teensyduino", ATTRS{product} == "Triple Serial", ATTRS{bcdDevice} == "0280", ATTRS{serial} == "9695040", SYMLINK += "beacon_brd_0"
ACTION == "add", SUBSYSTEMS == "usb", ATTRS{manufacturer} == "Teensyduino", ATTRS{product} == "Triple Serial", ATTRS{bcdDevice} == "0275", ATTRS{serial} == "9241440", SYMLINK += "beacon_brd_1"

Launch file still doesn't run properly, but basically this node:
subscribes to both of the beacon deployment boards' solenoid_pos and combines these into the topic (robot_ns)/beacon/solenoid_pos topic
publishes tp both boards' release_beacon topics by creating a callback for the /(robot_ns)/beacon/release_beacon topic
creates rosserial connections to both boards that should be namespaced something like: /(robot_ns)/beacon_board_x
then beacons can be dropped by publishing to beacon/release beacon, and if the channel is 0-3, beacons 0-3 on board 0 will be dropped
if the channel is 4-7, then one of the beacons in channel 0-3 in board 1 will be dropped
otherwise a warning will be given that the channel was out of range
