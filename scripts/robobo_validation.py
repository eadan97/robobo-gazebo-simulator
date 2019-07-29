#!/usr/bin/env python

# /*******************************************************************************
#  *
#  *   Copyright 2019, Manufactura de Ingenios Tecnológicos S.L. 
#  *   <http://www.mintforpeople.com>
#  *
#  *   Redistribution, modification and use of this software are permitted under
#  *   terms of the Apache 2.0 License.
#  *
#  *   This software is distributed in the hope that it will be useful,
#  *   but WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND; without even the implied
#  *   warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  *   Apache 2.0 License for more details.
#  *
#  *   You should have received a copy of the Apache 2.0 License along with    
#  *   this software. If not, see <http://www.apache.org/licenses/>.
#  *
#  ******************************************************************************/

import time as delay
# ROS libraries
import rospy

# ROS messages
from std_msgs.msg import Int32, Int16, Int8
# from sensor_msgs.msg import Range
from robobo_msgs.msg import IRs
from robobo_msgs.srv import MoveWheels, MovePanTilt
from random import randint

# Class that does all the work
class ROBOBO_VALIDATION(object):

    # Initialize ROS publishers, subscribers and variables
    def __init__(self, robobo_name='robot'):

        self.robobo_name = robobo_name

        # Initializes and cleanup ROS node
        rospy.init_node('RoboboValidation', anonymous=True)
        # ROS subscribers
        rospy.Subscriber(str(self.robobo_name) + '/irs', IRs, self.readIRS)

    # MoveWheels Ros service client
    def moveWheels(self, rs, ls, time):
        rospy.wait_for_service(str(self.robobo_name) + '/moveWheels')
        move_wheels = rospy.ServiceProxy(str(self.robobo_name) + '/moveWheels', MoveWheels)
        mw = move_wheels(Int8(ls), Int8(rs), Int32(time * 1000), Int16(0))
        if mw.error == 1:
            print('MoveWheels service call failed')

    def moveWheelsByTime(self, rs, ls, time):
        rospy.wait_for_service(str(self.robobo_name) + '/moveWheels')
        move_wheels = rospy.ServiceProxy(str(self.robobo_name) + '/moveWheels', MoveWheels)
        mw = move_wheels(Int8(ls), Int8(rs), Int32(time * 1000), Int16(0))
        delay.sleep(time)
        if mw.error == 1:
            print('MoveWheels service call failed')

    # MovePanTilt Ros service client
    def movePanTilt(self, pp, ps, tp, ts):
        rospy.wait_for_service(str(self.robobo_name) + '/movePanTilt')
        move_pan_tilt = rospy.ServiceProxy(str(self.robobo_name) + '/movePanTilt', MovePanTilt)
        mpt = move_pan_tilt(Int16(pp), Int8(ps), Int16(0), Int16(tp), Int8(ts), Int16(0))
        if mpt.error == 1:
            print('MovePanTilt service call failed')

    def readIRS(self, irs):
        self.FrontC = irs.FrontC.range
        self.FrontLL = irs.FrontLL.range
        self.FrontRR = irs.FrontRR.range
        self.BackC = irs.BackC.range
        self.BackL = irs.BackL.range
        self.BackR = irs.BackR.range

    def exploreIR(self, speed, maxIR):
        randomIni = 1
        randomFin = 3
        if self.FrontC > maxIR:
            self.moveWheels(0, 0, 0.1)
            self.moveWheelsByTime(speed * (-1), speed * (-1), randint(randomIni, randomFin))
            self.moveWheelsByTime(speed * (-1), speed, randint(randomIni, randomFin))
        elif self.FrontLL > maxIR:
            self.moveWheels(0, 0, 0.1)
            self.moveWheelsByTime(speed * (-1), speed * (-1), randint(randomIni, randomFin))
            self.moveWheelsByTime(speed * (-1), speed, randint(randomIni, randomFin))
        elif self.FrontRR > maxIR:
            self.moveWheels(0, 0, 0.1)
            self.moveWheelsByTime(speed * (-1), speed * (-1), randint(randomIni, randomFin))
            self.moveWheelsByTime(speed, speed * (-1), randint(randomIni, randomFin))
        elif self.BackC > maxIR:
            self.moveWheels(0, 0, 0.1)
            self.moveWheelsByTime(speed, speed, randint(randomIni, randomFin))
        elif self.BackL > maxIR:
            self.moveWheels(0, 0, 0.1)
            self.moveWheelsByTime(speed, speed, randint(randomIni, randomFin))
        elif self.BackR > maxIR:
            self.moveWheels(0, 0, 0.1)
            self.moveWheelsByTime(speed, speed, randint(randomIni, randomFin))

    def run(self):

        SPEED = 15
        MAX_IR = 250
        # Time while Robobo will be exploring
        EX_TIME = 30

        self.movePanTilt(180, 15, 85, 15)

        end_time = rospy.get_time() + EX_TIME
        self.moveWheels(SPEED, SPEED, 100)

        while rospy.get_time() < end_time:
            self.exploreIR(SPEED, MAX_IR)
            self.moveWheels(SPEED, SPEED, 100)

        self.moveWheels(0, 0, 1)

def main():

    instance = ROBOBO_VALIDATION()
    instance.run()

    print
    "Shutting down Robobo Validation"


if __name__ == '__main__':
    main()
