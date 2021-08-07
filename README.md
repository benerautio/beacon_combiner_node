# beacon_combiner node
## Exposes two beacon deployment boards as one 
* Must setup udev rules for boards for proper rosserial connection. Included in this repo, but must edit the serial number for different boards.
* Can see the serial number by running: udevadm info -a -p  $(udevadm info -q path -n /dev/ttyACM0)

This node:
* subscribes to both of the beacon deployment boards' solenoid_pos topics and combines these into the topic (robot_ns)/beacon/solenoid_pos topic
* publishes to both boards' release_beacon topics by creating a callback for the /(robot_ns)/beacon/release_beacon topic
* creates rosserial connections to both boards that should be namespaced something like: /(robot_ns)/beacon_board_x, with x either 0 or 1
* then beacons can be dropped by publishing to beacon/release beacon, and if the channel is 0-3, corresponding beacons 0-3 on board 0 will be dropped
* if the channel is 4-7, then one of the corresponding beacons on channels 0-3 on board 1 will be dropped
* if the channel is out of range, a warning will be logged and no beacon will drop
