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
#include </home/martin/ros_ws/src/iiwa_stack/iiwa_ros/include/iiwa_ros.h>

using namespace std;


void moveToStart(trajectory_msgs::JointTrajectory &new_trajectory)
{
    trajectory_msgs::JointTrajectoryPoint trajectory_start_point;
    rosbag::Bag bag;
    bag.open("/home/martin/bagdump/test.bag", rosbag::bagmode::Read);
    ROS_INFO("moveToStart opening the bag");

    std::vector<std::string> topics;
    std::string my_topic = "/iiwa/joint_states";
    topics.push_back(my_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    new_trajectory.points.clear();
    new_trajectory.header.stamp = ros::Time::now();
    trajectory_start_point.time_from_start = ros::Duration(2);

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
                {
                    if (m.getTopic() == my_topic)
                    {
                        sensor_msgs::JointState::ConstPtr i = m.instantiate<sensor_msgs::JointState>();
                        if (i != NULL)
                        {

                            for (int k = 0; k < 7; k++)
                            {
                                trajectory_start_point.positions.push_back(i->position[k]);
                                //trajectory_start_point.velocities.push_back(1);
                            }
                            new_trajectory.points.push_back(trajectory_start_point);
                            bag.close();
                            return;
                        }
                    }
                }

    return;
}

void moveToHome(trajectory_msgs::JointTrajectory &new_trajectory)
{
    trajectory_msgs::JointTrajectoryPoint trajectory_home_point;
    new_trajectory.points.clear();
    new_trajectory.header.stamp = ros::Time::now();
    trajectory_home_point.time_from_start = ros::Duration(2);

    for (int k = 0; k < 7; k++)
    {
        trajectory_home_point.positions.push_back(0);
        //trajectory_start_point.velocities.push_back(1);
    }
    trajectory_home_point.positions[6] = 1;
    new_trajectory.points.push_back(trajectory_home_point);

    return;
}

void execBagTrajectory(trajectory_msgs::JointTrajectory &new_trajectory)
{
    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    rosbag::Bag bag;
    bag.open("/home/martin/bagdump/test.bag", rosbag::bagmode::Read);
    ROS_INFO("Opening Bag Trajectory");
    iiwa_msgs::JointPosition my_joint_position = iiwa_ros:: jointQuantityFromDouble(0.0);
    iiwa_ros::iiwaRos my_iiwa_ros_object;
    ros::Duration tfs;
    std::vector<std::string> topics;
    std::string my_topic = "/iiwa/joint_states";
    topics.push_back(my_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    new_trajectory.points.clear();
    new_trajectory.header.stamp = ros::Time::now();
    trajectory_point.time_from_start = ros::Duration(0.0);

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
                {
                    if (m.getTopic() == my_topic)
                    {
                        sensor_msgs::JointState::ConstPtr i = m.instantiate<sensor_msgs::JointState>();
                        if (i != NULL)
                        {

                            for (int k = 0; k < 7; k++)
                            {
                                trajectory_point.positions.push_back(i->position[k]);
                                //trajectory_start_point.velocities.push_back(1);
                            }
                            tfs = trajectory_point.time_from_start;
                            trajectory_point.time_from_start = tfs + ros::Duration(0.02);
                            new_trajectory.points.push_back(trajectory_point);
                            trajectory_point.positions.clear();

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

    int jnum;

    while (ros::ok()) {
        // ros::spinOnce();

        cout<<"1: Start Pt (Demo), 3: Move to Home, 5: Execute Demo ";
        cin>>jnum;

        if (jnum ==1)
        {
            ROS_INFO("Moving to Bag Start Pt.");
            moveToStart(new_trajectory);
            pub.publish(new_trajectory);
        }

        if (jnum ==3)
        {
            ROS_INFO("Moving Home");
            moveToHome(new_trajectory);
            pub.publish(new_trajectory);
        }

        if (jnum ==5)
        {
            ROS_INFO("Executing Trajectory");
            execBagTrajectory(new_trajectory);
            pub.publish(new_trajectory);
        }

        sleep(1);
    }


    return 0;
}

