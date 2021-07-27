#!/usr/bin/env python

#TODO: fix for if solenoid status stops publishing??

import rospy
import numpy as np
from estop_msgs.msg import ServoStatus, SetChannel
from std_msgs.msg import Bool

class COMBINER:
    def __init__(self, robot_ns, num_beacons_brd_0, num_beacons_brd_1):

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
        self.sol_state = [0,0,0,0,0,0,0,0]

        # need a variable to keep track of how many beacons are dropped
        self.drop_time = 0
        self.dropper_timeout = rospy.Time(5)
        self.drop_in_progress = False
        self.dropped_beacons = 0
        #Available = 0, Bad = -1, Dropped = 1. Mark unloaded beacons as dropped
        self.num_beacons_0 = num_beacons_brd_0
        self.num_beacons_1 = num_beacons_brd_1
        self.beacons_per_board = 4
        self.beacon_state = np.ones(2*self.beacons_per_board, dtype=int)
        self.beacon_state[0:self.num_beacons_0] = np.zeros(self.num_beacons_0)
        self.beacon_state[self.beacons_per_board:(self.beacons_per_board + self.num_beacons_1)] = np.zeros(self.num_beacons_1)
        self.current_beacon = self.select_beacon()

    #gets executed whenever relay state changes
    def deployment_check_cb(self, msg):
        pass

    def get_beacon_status(self):
        return self.beacon_state[self.current_beacon]

    #subscribers to individual solenoid status should publish to the combined topic
    def sol_stat_brd_0_cb(self, msg):
        self.sol_state[:4] = msg.position[:4]
        combined_sol_status = ServoStatus()
        combined_sol_status.position = self.sol_state
        self.pub_sol_status.publish(combined_sol_status)

        #check feedback for drop in progress, only if drop is in progress?
        if(self.drop_in_progress):
            if (msg.position[self.current_beacon] >= 100):
                rospy.loginfo("Successful drop, marking beacon as dropped and selecting new beacon")
                self.drop_in_progress = False
                self.dropped_beacons+=1
                self.beacon_state[self.current_beacon] = 1
                #successful drop!

            elif (rospy.Time.now() - self.drop_time > self.dropper_timeout):
                #unsuccessful drop
                rospy.loginfo("Unsuccessful drop, marking beacon as bad and selecting new beacon")
                self.beacon_state[self.current_beacon] = -1
                self.drop_in_progress = False
                
    def select_beacon(self):
        for beacon in range(0,len(self.beacon_state)):
            if self.beacon_state[beacon] == 0:
                self.current_beacon = beacon
                return
        rospy.loginfo("No more Beacons to drop!")

    
    #subscribers to individual solenoid status should publish to the combined topic
    def sol_stat_brd_1_cb(self, msg):
        self.sol_state[4:] = msg.position[:4]
        combined_sol_status = ServoStatus()
        combined_sol_status.position = self.sol_state
        self.pub_sol_status.publish(combined_sol_status)

         #check feedback for drop in progress, only if drop is in progress?
        if(self.drop_in_progress):
            if (msg.position[self.current_beacon - self.beacons_per_board] >= 100):
                rospy.loginfo("Successful drop, marking beacon as dropped and selecting new beacon")
                self.drop_in_progress = False
                self.dropped_beacons+=1
                self.beacon_state[self.current_beacon] = 1
                #successful drop!

            elif (rospy.Time.now() - self.drop_time > self.dropper_timeout):
                #unsuccessful drop
                rospy.loginfo("Unsuccessful drop, marking beacon as bad and selecting new beacon")
                self.beacon_state[self.current_beacon] = -1
                self.drop_in_progress = False
    
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

            #select a beacon based on status, problem here
            self.select_beacon()
            drop.id = self.current_beacon
            if (self.get_beacon_status() == 0):
                self.drop_in_progress = True
                self.drop_time = rospy.Time.now()
                self.combined_release_cb(drop)
            else:
                rospy.logwarn("Caught bad beacon deployment, no beacon dropped")

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
        else:
            rospy.logwarn('Undefined Solenoid Channel, no dropped beacon')
        

def main():
    rospy.init_node('beacon_combiner')
    robot_ns = rospy.get_param('~vehicle_name', 'H01')
    num_beacons_brd_0 = rospy.get_param('~num_beacons_brd_0', 3)
    num_beacons_brd_1 = rospy.get_param('~num_beacons_brd_1', 3)
    beacon_combiner = COMBINER(robot_ns, num_beacons_brd_0, num_beacons_brd_1)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInternalException:
        pass