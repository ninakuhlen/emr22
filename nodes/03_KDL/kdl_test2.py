#!/usr/bin/env python
# kdl_test.py
# ------------------------------------
# edited WHS, OJ , 9.2.2022 #
# -------------------------------------
# Die hier beschriebene Methode ist nur eine – relativ naive – Möglichkeit
# die TCP-Position eines Roboterarms zu steuern.
# Diese einfache Implementierung zielt darauf ab ein Verständnis

# $1 roslaunch ur5-joint-position-control ur5_gazebo_joint_position_control.launch

import rospy
# from std_msgs.msg import String
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
import PyKDL as kdl


shoulder_pan_joint_position = 0.0
shoulder_lift_joint_position = 0.0
elbow_joint_position = 0.0
wrist_1_joint_position = 0.0
wrist_2_joint_position = 0.0
wrist_3_joint_position = 0.0

jnt_pos_start = [shoulder_pan_joint_position,
                 shoulder_lift_joint_position,
                 elbow_joint_position,
                 wrist_1_joint_position,
                 wrist_2_joint_position,
                 wrist_3_joint_position]


def get_shoulder_pan_joint_position(JointControllerStateMsg):
    # callback for subscribed message
    jnt_pos_start[0] = JointControllerStateMsg.set_point


def get_shoulder_lift_joint_position(JointControllerStateMsg):
    # callback for subscribed message
    jnt_pos_start[1] = JointControllerStateMsg.set_point


def get_elbow_joint_position(JointControllerStateMsg):
    # callback for subscribed message
    jnt_pos_start[2] = JointControllerStateMsg.set_point


def get_wrist1_joint_position(JointControllerStateMsg):
    # callback for subscribed message
    jnt_pos_start[3] = JointControllerStateMsg.set_point


def get_wrist2_joint_position(JointControllerStateMsg):
    # callback for subscribed message
    jnt_pos_start[4] = JointControllerStateMsg.set_point


def get_wrist3_joint_position(JointControllerStateMsg):
    # callback for subscribed message
    jnt_pos_start[5] = JointControllerStateMsg.set_point


if __name__ == '__main__':
    rospy.init_node('py_kdl_node', anonymous=True)
    shoulder_pan_sub = rospy.Subscriber("/shoulder_pan_joint_position_controller/state",
                                        JointControllerState,
                                        get_shoulder_pan_joint_position)
    shoulder_lift_sub = rospy.Subscriber("/shoulder_lift_joint_position_controller/state",
                                         JointControllerState,
                                         get_shoulder_lift_joint_position)
    elbow_sub = rospy.Subscriber("/elbow_joint_position_controller/state",
                                 JointControllerState,
                                 get_elbow_joint_position)

    # $ rostopic info /wrist1_joint_position_controller/state
    # ERROR: Unknown topic /wrist1_joint_position_controller/state

    wrist1_sub = rospy.Subscriber("/wrist1_joint_position_controller/state",
                                  JointControllerState,
                                  get_wrist1_joint_position)
    wrist2_sub = rospy.Subscriber("/wrist2_joint_position_controller/state",
                                  JointControllerState,
                                  get_wrist2_joint_position)
    wrist3_sub = rospy.Subscriber("/wrist3_joint_position_controller/state",
                                  JointControllerState,
                                  get_wrist3_joint_position)

    # Create a bunch of Publishers
    shoulder_pan_pub = rospy.Publisher("/shoulder_pan_joint_position_controller/command",
                                       Float64,
                                       queue_size=20)
    shoulder_lift_pub = rospy.Publisher("/shoulder_lift_joint_position_controller/command",
                                        Float64,
                                        queue_size=20)
    elbow_pub = rospy.Publisher("/elbow_joint_position_controller/command",
                                Float64,
                                queue_size=20)
    wrist1_pub = rospy.Publisher("/wrist1_joint_position_controller/command",
                                 Float64,
                                 queue_size=20)
    wrist2_pub = rospy.Publisher("/wrist2_joint_position_controller/command",
                                 Float64,
                                 queue_size=20)
    wrist3_pub = rospy.Publisher("/wrist3_joint_position_controller/command",
                                 Float64,
                                 queue_size=20)

    while not rospy.is_shutdown():
        # Compute current tcp position
        # KDL::Frame tcp_pos_start;

        print("Joint Pos ", jnt_pos_start[0],
              jnt_pos_start[1], jnt_pos_start[2],
              jnt_pos_start[3], jnt_pos_start[4],
              jnt_pos_start[5])

        # new_pose = [0.0, 0.0, 1.0, 0.0, 0.0, 0.1]
        # tcp_pos_start = kdl.Frame()
        new_pos = jnt_pos_start[0] - 0.1
        shoulder_pan_pub.publish(new_pos)

        new_pos = jnt_pos_start[1] - 0.1
        shoulder_lift_pub.publish(new_pos)

        new_pos = jnt_pos_start[2] - 0.1
        elbow_pub.publish(new_pos)
        
        new_pos = jnt_pos_start[3] + 0.1
        wrist1_pub.publish(new_pos)

        new_pos = jnt_pos_start[4] + 0.1
        wrist2_pub.publish(new_pos)
        
        new_pos = jnt_pos_start[5] + 0.1
        wrist3_pub.publish(new_pos)

        rospy.sleep(1)  # Sleeps for 1 sec


        # fk_solver.JntToCart(jnt_pos_start, tcp_pos_start);

        """print("Current tcp Position/Twist KDL:")
        print("Position: %f %f %f",
              tcp_pos_start.p(0),
              tcp_pos_start.p(1),
              tcp_pos_start.p(2))

        print("Orientation: %f %f %f",
              tcp_pos_start.M(0, 0),
              tcp_pos_start.M(1, 0),
              tcp_pos_start.M(2, 0))"""



"""
void get_shoulder_pan_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {
    jnt_pos_start(0) = ctr_msg->process_value;
}

float compute_linear(double q_start, double q_goal, float t, float t_max) {
    return((q_goal - q_start) * (t/t_max) + q_start);
}

void get_goal_tcp_and_time(KDL::Frame tcp_pos_start, KDL::Vector* vec_tcp_pos_goal, float* t_max) {

    std::cout << "Please define the offset (from startpose) you want to move for each axis and the time in which the motion should be completed:\n";

        //Get user input
        float x,y,z;
        std::cout << "x:";
        std::cin >> x;
        std::cout << "y:";
        std::cin >> y;
        std::cout << "z:";
        std::cin >> z;
        std::cout << "Time:";
        std::cin >> (*t_max);

        //Compute goal position
        (*vec_tcp_pos_goal)(0) = (tcp_pos_start.p(0) + x);
        (*vec_tcp_pos_goal)(1) = (tcp_pos_start.p(1) + y);
        (*vec_tcp_pos_goal)(2) = (tcp_pos_start.p(2) + z);
}

const int loop_rate_val = 100;

int main(int argc, char **argv)
{
    std::string urdf_path = ros::package::getPath("ur5-joint-position-control");
    if(urdf_path.empty()) {
        ROS_ERROR("ur5-joint-position-control package path was not found");
    }
    urdf_path += "/urdf/ur5_jnt_pos_ctrl.urdf";
    ros::init(argc, argv, "tcp_control");

    ros::NodeHandle n;

    ros::Rate loop_rate(loop_rate_val);

    //Create subscribers for all joint states
    ros::Subscriber shoulder_pan_joint_sub = n.subscribe("/shoulder_pan_joint_position_controller/state", 1000, get_shoulder_pan_joint_position);
    ros::Subscriber shoulder_lift_joint_sub = n.subscribe("/shoulder_lift_joint_position_controller/state", 1000, get_shoulder_lift_joint_position);
    ros::Subscriber elbow_joint_sub = n.subscribe("/elbow_joint_position_controller/state", 1000, get_elbow_joint_position);
    ros::Subscriber wrist_1_joint_sub = n.subscribe("/wrist_1_joint_position_controller/state", 1000, get_wrist_1_joint_position);
    ros::Subscriber wrist_2_joint_sub = n.subscribe("/wrist_2_joint_position_controller/state", 1000, get_wrist_2_joint_position);
    ros::Subscriber wrist_3_joint_sub = n.subscribe("/wrist_3_joint_position_controller/state", 1000, get_wrist_3_joint_position);

    //Create publishers to send position commands to all joints
    ros::Publisher joint_com_pub[6];
    joint_com_pub[0] = n.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command", 1000);
    joint_com_pub[1] = n.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command", 1000);
    joint_com_pub[2] = n.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command", 1000);
    joint_com_pub[3] = n.advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command", 1000);
    joint_com_pub[4] = n.advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command", 1000);
    joint_com_pub[5] = n.advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command", 1000);

    //Parse urdf model and generate KDL tree
    KDL::Tree ur5_tree;
    if (!kdl_parser::treeFromFile(urdf_path, ur5_tree)){
        ROS_ERROR("Failed to construct kdl tree");
           return false;
    }

    //Generate a kinematic chain from the robot base to its tcp
    KDL::Chain ur5_chain;
    ur5_tree.getChain("base_link", "wrist_3_link", ur5_chain);

    //Create solvers
    KDL::ChainFkSolverPos_recursive fk_solver(ur5_chain);
    KDL::ChainIkSolverVel_pinv vel_ik_solver(ur5_chain, 0.0001, 1000);
    KDL::ChainIkSolverPos_NR ik_solver(ur5_chain, fk_solver, vel_ik_solver, 1000);

    //Make sure we have received proper joint angles already
    for(int i=0; i< 2; i++) {
        ros::spinOnce();
         loop_rate.sleep();
    }

    const float t_step = 1/((float)loop_rate_val);
    int count = 0;
    while (ros::ok()) {

        //Compute current tcp position
        KDL::Frame tcp_pos_start;
        fk_solver.JntToCart(jnt_pos_start, tcp_pos_start);

        ROS_INFO("Current tcp Position/Twist KDL:");
        ROS_INFO("Position: %f %f %f", tcp_pos_start.p(0), tcp_pos_start.p(1), tcp_pos_start.p(2));
        ROS_INFO("Orientation: %f %f %f", tcp_pos_start.M(0,0), tcp_pos_start.M(1,0), tcp_pos_start.M(2,0));

        //get user input
        float t_max;
        KDL::Vector vec_tcp_pos_goal(0.0, 0.0, 0.0);
        get_goal_tcp_and_time(tcp_pos_start, &vec_tcp_pos_goal, &t_max);

        KDL::Frame tcp_pos_goal(tcp_pos_start.M, vec_tcp_pos_goal);

        //Compute inverse kinematics
        KDL::JntArray jnt_pos_goal(Joints);
        ik_solver.CartToJnt(jnt_pos_start, tcp_pos_goal, jnt_pos_goal);

        float t = 0.0;
        while(t<t_max) {
            std_msgs::Float64 position[6];
            //Compute next position step for all joints
            for(int i=0; i<Joints; i++) {
                position[i].data = compute_linear(jnt_pos_start(i), jnt_pos_goal(i), t, t_max);
                joint_com_pub[i].publish(position[i]);
            }

            ros::spinOnce();
            loop_rate.sleep();
            ++count;
            t += t_step;
        }
    }
    return 0;
} """
