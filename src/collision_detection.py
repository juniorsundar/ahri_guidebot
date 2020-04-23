import pybullet as p
import time
import rospy
from cairo_simulator.Simulator import Simulator, SimObject, ASSETS_PATH
from cairo_simulator.Manipulators import Sawyer
import numpy as np
import pdb
from std_msgs.msg import Float32MultiArray, Empty, String, Int16

stop_move_state = None


def collision(obj_pos,arm_pos,threshold):
    '''
    check if arm is entering obstacle region.

    Return:
        boolean value for collision status
    '''
    arm_pos = list(arm_pos[0])
    print(arm_pos[0])
    for ob in obj_pos:
        dist = np.sqrt((ob[0]-arm_pos[0]) ** 2 + (ob[1] - arm_pos[1]) ** 2 + (ob[2] - arm_pos[2]) ** 2)
        print("distance ", dist)
        if(dist < threshold):
            print("Entered collision zone")
            return True
    return False

def stop_moving(data):
    '''
    data: array of arm position where it entered collision zone.
    '''
    print("stoped moving at ",data.data)
    # rospy.loginfo(rospy.get_caller_id() + "Stop moving arm at",data.data)
    

def main():
    global stop_move_state
    rospy.init_node("CAIRO_Sawyer_Simulator")
    use_real_time = True

    # stop node
    stop_move_state = rospy.Publisher('stop',Float32MultiArray,queue_size=10)
    rospy.Subscriber('stop',Float32MultiArray,stop_moving)

    sim = Simulator() # Initialize the Simulator

    # Add a table and a Sawyer robot
    table = SimObject("Table", ASSETS_PATH + 'table.sdf', (0.9, 0, 0), (0, 0, 1.5708)) # Table rotated 90deg along z-axis
    sawyer_robot = Sawyer("sawyer0", 0, 0, 0.8)

    obj_x = np.random.uniform(0.67,.8)
    obj_y = np.random.uniform(-0.5,0.5)
    obj_pos = ((obj_x,0,0.55),(0.99, obj_y, .55))


    threshold = 0.3

    
    sim_obj1 = SimObject('cube2', 'cube_small.urdf', obj_pos[0])
    sim_obj4 = SimObject('cube3', 'cube_small.urdf', obj_pos[1])

    joint_config = sawyer_robot.solve_inverse_kinematics([0.9,0,1.5], [0,0,0,1])


    # different x position to sweep across table
    y1 = np.linspace(-0.9,0,5) # move up on table
    y2 = np.linspace(0.1,0.9,5) # move down on table
    y = np.append(y1,y2)

    x = np.linspace(0.3,1,10)
    z = np.array([0.7]*10)
    c = 0
    for i,j,k in zip(x,y,z):
        # print("x and y pos: {} {} ".format(i,j))
        joint_config = sawyer_robot.solve_inverse_kinematics([i,j,k],[0.8,0.5,j])
        print(c)
        c += 1
        joint_config[5] = 1 # gripper position. giving roll value 1 will rotate along x-axis.
        sawyer_robot.move_to_joint_pos(joint_config)
        link_state = p.getLinkState(sawyer_robot._simulator_id,sawyer_robot._end_effector_link_index)
        print("EOF pos ",link_state[0])
        if(collision(obj_pos,link_state,threshold) == True):
            stop_move_state.publish(Float32MultiArray(data=[i,j,k]))
            break
        time.sleep(5)
        
    print("done moving")
    # pdb.set_trace()


    # Loop until someone shuts us down
    while rospy.is_shutdown() is not True:
        sim.step()
    p.disconnect()


if __name__ == "__main__":
    main()
