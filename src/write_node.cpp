#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <sstream>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <boost/foreach.hpp>
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointPositionVelocity.h"
#include "iiwa_msgs/JointQuantity.h"
#include "iiwa_msgs/JointVelocity.h"
#include "iiwa_ros.h"


//#include </home/martin/ros_ws/src/iiwa_stack/iiwa_ros/include/iiwa_ros.h>

using namespace std;


void clearPositions(iiwa_msgs::JointPosition &new_trajectory)
{

}
void moveToStart(iiwa_msgs::JointPosition &joint_pos, ros::Publisher &pub)
{
    rosbag::Bag bag;
    bag.open("/home/martin/bagdump/test.bag", rosbag::bagmode::Read);
    ROS_INFO("moveToStart opening the bag");
    std::vector<std::string> topics;
    std::string my_topic = "/iiwa/joint_states";
    topics.push_back(my_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
                {
                    if (m.getTopic() == my_topic)
                    {
                        sensor_msgs::JointState::ConstPtr i = m.instantiate<sensor_msgs::JointState>();
                        if (i != NULL)
                        {


                            joint_pos.position.a1 = i->position[0];
                            joint_pos.position.a2 = i->position[1];
                            joint_pos.position.a3 = i->position[2];
                            joint_pos.position.a4 = i->position[3];
                            joint_pos.position.a5 = i->position[4];
                            joint_pos.position.a6 = i->position[5];
                            joint_pos.position.a7 = i->position[6];
                            pub.publish(joint_pos);

                            bag.close();
                            return;
                        }
                    }
                }

    return;
}

void moveToHome(iiwa_msgs::JointPosition &joint_pos, ros::Publisher &pub)
{


    joint_pos.position.a1 = 0;
    joint_pos.position.a2 = 0;
    joint_pos.position.a3 = 0;
    joint_pos.position.a4 = 0;
    joint_pos.position.a5 = 0;
    joint_pos.position.a6 = 0;
    joint_pos.position.a7 = 0;
    pub.publish(joint_pos);
    return;
}

void execBagTrajectory(iiwa_msgs::JointPosition &joint_pos, ros::Publisher &pub)
{
    rosbag::Bag bag;
    bag.open("/home/martin/bagdump/test.bag", rosbag::bagmode::Read);
    ROS_INFO("Opening Bag Trajectory");
    ros::Duration tfs;
    std::vector<std::string> topics;
    std::string my_topic = "/iiwa/joint_states";
    topics.push_back(my_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
                {
                    if (m.getTopic() == my_topic)
                    {
                        sensor_msgs::JointState::ConstPtr i = m.instantiate<sensor_msgs::JointState>();
                        if (i != NULL)
                        {

                            joint_pos.position.a1 = i->position[0];
                            joint_pos.position.a2 = i->position[1];
                            joint_pos.position.a3 = i->position[2];
                            joint_pos.position.a4 = i->position[3];
                            joint_pos.position.a5 = i->position[4];
                            joint_pos.position.a6 = i->position[5];
                            joint_pos.position.a7 = i->position[6];
                            pub.publish(joint_pos);
                            ros::Duration(0.02).sleep();

                        }
                    }
                }
    bag.close();
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_sender");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("iiwa/PositionJointInterface_trajectory_controller/command", 1);
    ros::Rate sleep_timer(0.1); // update rate
    iiwa_ros::iiwaRos ok;

    iiwa_msgs::JointPosition joint_pos;


    int jnum;

    while (ros::ok()) {
       // ros::spinOnce();

        cout<<"1: Start Pt (Demo), 3: Move to Home, 5: Execute Demo ";
        cin>>jnum;

        if (jnum ==1)
        {
            ROS_INFO("Moving to Bag Start Pt.");
            moveToStart(joint_pos, pub);
        }

        if (jnum ==3)
        {
            ROS_INFO("Moving Home");
            moveToHome(joint_pos, pub);
        }

        if (jnum ==5)
        {
            ROS_INFO("Executing Trajectory");
            execBagTrajectory(joint_pos, pub);
        }

        sleep(1);
    }
 
 
    return 0;
}

