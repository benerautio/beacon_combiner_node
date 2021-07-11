#!/usr/bin/env python

import rospy
import numpy as np
from estop_msgs.msg import ServoStatus, SetChannel
from std_msgs.msg import Bool

class COMBINER:
    def __init__(self, robot_ns):

        #Subscribe to both boards solenoid status
        self.sub_sol_brd_0 = rospy.Subscriber('/'+robot_ns+'/beacon_board_0/solenoid_pos', ServoStatus, self.sol_stat_brd_0_cb)
        self.sub_sol_brd_1 = rospy.Subscriber('/'+robot_ns+'/beacon_board_1/solenoid_pos', ServoStatus, self.sol_stat_brd_1_cb)

        #publisher to combined solenoid status topic 
        self.pub_sol_status = rospy.Publisher('/'+robot_ns+'/beacon/solenoid_pos', ServoStatus, queue_size=10)
        #create subscriber to this topic as well so that we can know if the beacon deployed successfully
        self.sub_deployment_check = rospy.Subscriber('/'+robot_ns+'/beacon/solenoid_pos', ServoStatus, self.deployment_check_cb)

        #subscribe to combined release_beacon topic
        self.sub_drop = rospy.Subscriber('/'+robot_ns+'/beacon/release_beacon', SetChannel, self.combined_release_cb)

        #subscribe to base station, dan's topic to trigger beacon release
        self.sub_deploy = rospy.Subscriber('/'+robot_ns+'/deploy',Bool,self.deploy_cb)

        #create publisher to both boards' release_beacon topic
        self.pub_release_beacon_brd_0 = rospy.Publisher('/'+robot_ns+'/beacon_board_0/release_beacon',SetChannel,queue_size=10)
        self.pub_release_beacon_brd_1 = rospy.Publisher('/'+robot_ns+'/beacon_board_1/release_beacon',SetChannel,queue_size=10)

        # some variables for solenoid state
        # 8 beacons, is only 1 while solenoid is on, then is zero after beacon drops. 
        self.sol_state = [0,0,0,0,0,0,0,0]

        #This is to check if a beacon deployed successfully. Whenever relay state changes to 1, 
        #corresponding element in the array becomes 1, indicating a successful drop
        self.dep_check = np.array([0,0,0,0,0,0,0,0])

        # Therefore need a variable to keep track of how many beacons are dropped
        # When this is 7, all beacons are dropped.
        self.dropped_beacons = 0
        self.beacons_per_board = 4

    #gets executed whenever relay state changes, updates array of deployed beacons, deployed = 1
    def deployment_check_cb(self, msg):
        self.dep_check += msg.position

    #subscribers to individual solenoid status should publish to the combined topic
    def sol_stat_brd_0_cb(self, msg):
        self.sol_state[:4] = msg.position[:4]
        combined_sol_status = ServoStatus()
        combined_sol_status.position = self.sol_state
        self.pub_sol_status.publish(combined_sol_status)
    
    #subscribers to individual solenoid status should publish to the combined topic
    def sol_stat_brd_1_cb(self, msg):
        self.sol_state[4:] = msg.position[:4]
        combined_sol_status = ServoStatus()
        combined_sol_status.position = self.sol_state
        self.pub_sol_status.publish(combined_sol_status)
    
    def release_brd_0_cb(self, msg):
        rospy.logdebug("releasing a beacon on board 0")

    def release_brd_1_cb(self, msg):
        rospy.logdebug("releasing a beacon on board 1")

    def deploy_cb(self,msg):
        if msg.data == True:
            if (self.dropped_beacons > 7):
                rospy.logwarn("Tried to drop more than 8 beacons!")
                return 
            drop = SetChannel()
            drop.state = 1
            drop.id = self.dropped_beacons

            self.combined_release_cb(drop)

            #check if beacon deployed successfully
            if (self.dep_check[self.dropped_beacons] > 0):
                rospy.loginfo("beacon dropped successfully")
                self.dropped_beacons = self.dropped_beacons + 1
                rospy.loginfo("dropped beacons: %d" % self.dropped_beacons)
                if self.dropped_beacons == 4:
                    rospy.logwarn('beacon board 0 now empty')
                elif self.dropped_beacons >= 8:
                    rospy.logwarn('beacon board 1 now empty')
                    rospy.logwarn('No more beacons to drop')
            elif (self.dep_check[self.dropped_beacons] == 0):
                rospy.logwarn("beacon drop failed")
            else:
                # print(self.dep_check)
                # print(drop.id)
                # print(self.dep_check[drop.id])
                rospy.logwarn("tried to drop the same beacon twice")

    #this is what makes the two different topics behave as one
    #if the channel is 0-3, this goes to board 0. If it is 4-7, this goes to board 1
    def combined_release_cb(self, msg):
        if msg.id <= 3 and msg.id >= 0:
            rospy.loginfo("trying to release beacon...")
            self.pub_release_beacon_brd_0.publish(msg)
        elif msg.id <=7 and msg.id >= 4:
            rospy.loginfo("trying to release beacon...")
            msg.id=msg.id-self.beacons_per_board
            self.pub_release_beacon_brd_1.publish(msg)
            #msg.id+=self.beacons_per_board
        else:
            rospy.logwarn('Undefined Solenoid Channel, no dropped beacon')
        

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