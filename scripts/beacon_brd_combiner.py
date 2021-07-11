#!/usr/bin/env python

import rospy
from estop_msgs.msg import ServoStatus, SetChannel
from std_msgs.msg import Bool

class COMBINER:
    def __init__(self, robot_ns):

        #Subscribe to both boards solenoid status
        self.sub_sol_brd_0 = rospy.Subscriber('/'+robot_ns+'/beacon_board_0/solenoid_pos', ServoStatus, self.sol_stat_brd_0_cb)
        self.sub_sol_brd_1 = rospy.Subscriber('/'+robot_ns+'/beacon_board_1/solenoid_pos', ServoStatus, self.sol_stat_brd_1_cb)

        #publisher to combined solenoid status topic 
        self.pub_sol_status = rospy.Publisher('/'+robot_ns+'/beacon/solenoid_pos', ServoStatus, queue_size=10)

        #subscribe to combined release_beacon topic
        self.sub_drop = rospy.Subscriber('/'+robot_ns+'/beacon/release_beacon', SetChannel, self.combined_release_cb)

        #subscribe to base station
        #self.sub_deploy = rospy.Subscriber('/'+robot_ns+'/deploy',Bool,self.deploy_cb)

        #create publisher to both boards' release_beacon topic
        self.pub_release_beacon_brd_0 = rospy.Publisher('/'+robot_ns+'/beacon_board_0/release_beacon',SetChannel,queue_size=10)
        self.pub_release_beacon_brd_1 = rospy.Publisher('/'+robot_ns+'/beacon_board_1/release_beacon',SetChannel,queue_size=10)



        # some variables for solenoid state
        # 8 beacons, is only 1 while solenoid is on, then is zero after beacon drops. 
        self.sol_state = [0,0,0,0,0,0,0,0]
        # Therefore need a variable to keep track of how many beacons are dropped
        # When this is 7, all beacons are dropped.
        self.droppped_beacons = 0
        self.beacons_per_board = 4

    #callbacks
    #consider:
    #status callback updates corect element of array
    #publishing a drop to release_beacon drops correct beacon, and updates dropped_beacons. logwarn when none left

    #subscribers to individual solenoid status should publish to the combined topic
    def sol_stat_brd_0_cb(self, msg):
        self.sol_state[:3] = msg.position[:3]
        self.pub_sol_status.publish(self.sol_state)
    
    def sol_stat_brd_1_cb(self, msg):
        self.sol_state[4:] = msg.position[:3]
        self.pub_sol_status.publish(self.sol_state)
    
    def release_brd_0_cb(self, msg):
        rospy.logdebug("releasing beacon on board 0")

    def release_brd_1_cb(self, msg):
        rospy.logdebug("releasing beacon on board 1")

    def deploy_cb(self,msg):
        if msg.data == True:
            drop = SetChannel()
            drop.state = 1
            drop.id = self.dropped_beacons

            self.combined_release_cb(drop)

            self.droppped_beacons = self.dropped_beacons + 1
            if self.droppped_beacons == 4:
                rospy.logwarn('beacon board 0 now empty')
            elif self.droppped_beacons == 8:
                rospy.logwarn('beacon board 1 now empty')
            else:
                rospy.logwarn('No beacons to drop')

    #this is what makes the two different topics behave as one
    #if the channel is 0-3, this goes to board 0. If it is 4-7, this goes to board 1
    def combined_release_cb(self, msg):
        if msg.id <= 3 and msg.id >= 0:
            self.pub_release_beacon_brd_0.publish(msg)
        elif msg.id <=7 and msg.id >= 4:
            msg.id=msg.id-self.beacons_per_board
            self.pub_release_beacon_brd_1.publish(msg)
        else:
            rospy.logwarn('Undefined Solenoid Channel')
        

def main():
    rospy.init_node('beacon_combiner')
    robot_ns = rospy.get_param('~robot_ns', 'H01')
    beacon_combiner = COMBINER(robot_ns)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInternalException:
        pass