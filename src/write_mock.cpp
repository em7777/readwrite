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


using namespace std;
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"



int main(int argc, char **argv)
{

    ros::init(argc, argv, "traj_sender");

    ros::NodeHandle n;

//    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("iiwa/PositionJointInterface_trajectory_controller/command", 1);

    std_msgs::Float32 pi;
    pi.data=3.14;
    double theta[7];
    bool firstpt = true;
    ros::Duration tfs;
    trajectory_msgs::JointTrajectory new_trajectory;
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    new_trajectory.joint_names.push_back("iiwa_joint_1");
    new_trajectory.joint_names.push_back("iiwa_joint_2");
    new_trajectory.joint_names.push_back("iiwa_joint_3");
    new_trajectory.joint_names.push_back("iiwa_joint_4");
    new_trajectory.joint_names.push_back("iiwa_joint_5");
    new_trajectory.joint_names.push_back("iiwa_joint_6");
    new_trajectory.joint_names.push_back("iiwa_joint_7");
    new_trajectory.points.clear();
    trajectory_point1.positions.clear();



    rosbag::Bag bag;

    bag.open("/home/martin/bagdump/test.bag", rosbag::bagmode::Read);
    ROS_INFO("Opening the bag");

    std::vector<std::string> topics;
    std::string my_topic = "/iiwa/joint_states";
    topics.push_back(my_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    new_trajectory.header.stamp = ros::Time::now();
    trajectory_point1.time_from_start = ros::Duration(0.0);
    new_trajectory.points.clear();


    while (ros::ok()) {

        ros::spinOnce();

        BOOST_FOREACH(rosbag::MessageInstance const m, view)
                    {
                        if (m.getTopic() == my_topic)
                        {
                            sensor_msgs::JointState::ConstPtr i = m.instantiate<sensor_msgs::JointState>();
                            if (i != NULL)
                            {

                                for (int k = 0; k < 7; k++)
                                {
                                    trajectory_point1.positions.push_back(i->position[k]);
                                    trajectory_point1.velocities.push_back(i->velocity[k]);
                                }
                                tfs = trajectory_point1.time_from_start;
                                trajectory_point1.time_from_start = tfs + ros::Duration(0.02);
                                new_trajectory.points.push_back(trajectory_point1);
                                trajectory_point1.positions.clear();
                                trajectory_point1.velocities.clear();



                            }
                        }
                    }
        bag.close();

        pub.publish(new_trajectory);





//        ROS_INFO("waking from 1s sleep");
//
//        trajectory_point1.positions[1] = 1.0;
//        //new_trajectory.header.stamp = ros::Time::now();
//        //trajectory_point1.time_from_start = ros::Duration(2);
//        new_trajectory.points.clear();
//        new_trajectory.points.push_back(trajectory_point1);

    }


    return 0;
}


/*
 * Algorithm
 *
 * 1. Call write function with bagname, time as function param.
 * 2. Prompt
 * 3. Go to first position in file
 * 4. Prompt
 * 5. Execute trajectory
 *
 **
 *

 /*
 Method 1:

1. Open Rosbag
2. Read message (for n messages)
    Store POSITION of JOINTS directly into new_traj (one large memory)
    Publish the ONE message

    or

2. Store all positions in local memory
    load position into trajectory and publish each point
    sleep given time
    get next pointm load, publish etc.
 */

//


