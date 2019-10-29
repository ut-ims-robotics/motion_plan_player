#!/usr/bin/python

import sys
import time
import rospy
#import moveit_commander
#import kdl_parser_py
import xacro
import subprocess
import rospkg
from urdf_parser_py.urdf import URDF
from kdl_parser_py import urdf
import PyKDL as kdl
import numpy as np

def load_robot_description():
    xacro_path = rospkg.RosPack().get_path('franka_description')+'/robots/panda_arm.urdf.xacro'
    print xacro_path

    try:
        command_string = "rosrun xacro xacro --inorder "+ xacro_path
        robot_description = subprocess.check_output(command_string, shell=True, stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as process_error:
        rospy.logfatal('Failed to run xacro command with error: \n%s', process_error.output)
        sys.exit(1)

    rospy.set_param("/robot_description", robot_description)



def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

if __name__ == '__main__':

    base_link = 'panda_link0'
    end_link = 'panda_link7'
    try:
        rospy.init_node('jacobian_planner')

        load_robot_description()
        time.sleep(2)


        print("Get urdf model from parameter server")
        model = URDF.from_parameter_server()

        #print model
        
        print("Loading kdl tree using urdf from parameter server")
        ret, kdltree = urdf.treeFromUrdfModel(model, quiet=False)

        # Get the whole kinematic chain from kdl tree
        chain = kdltree.getChain(base_link, end_link)
            
        # Initialize kdl solver for jacobian calculation
        solver = kdl.ChainJntToJacSolver(chain)

        jnt_array = kdl.JntArray(chain.getNrOfSegments())


        # Get joints from the chain
        #for i in range(chain.getNrOfSegments()):
        #    print chain.getSegment(i).getJoint()


        jac = kdl.Jacobian(chain.getNrOfSegments())
#        print(dir(kdl))

        my_arr = np.zeros((6,7))

        if(solver.JntToJac(jnt_array, jac) == 0):
            rospy.loginfo("Sucessfully calculated jacobian")
            print jac[0,3]

            for i in range(0,6):
                for j in range(0,7):
                    my_arr[i,j] = jac[i,j]
                   # print(jac[i,j])
                   
            print(np.dot(my_arr , [1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
            
        else:
            rospy.logwarn("Failed to calculate jacobian")

        #while not rospy.is_shutdown():
        #    rospy.loginfo("Working...")
        #    rospy.sleep(0.5)

    except rospy.ROSInterruptException:
        pass
