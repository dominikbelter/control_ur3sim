#include "ros/ros.h"

#include "control_msgs/JointTrajectoryControllerState.h" //read state of the arm
#include "control_msgs/FollowJointTrajectoryAction.h" //set goal positions
#include "sensor_msgs/JointState.h"
#include "moveit_msgs/GetPositionIK.h"
#include "moveit_msgs/GetPositionFK.h"
#include "moveit_msgs/DisplayRobotState.h"

std::vector<double> currentPos = {0,0,0,0,0,0};

void urStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
    for (size_t jointNo=0; jointNo<6;jointNo++){
        ROS_INFO("Joint pos[%d]: %f", (int)jointNo, msg->actual.positions[jointNo]);
        currentPos[jointNo] = msg->actual.positions[jointNo];
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur3_control");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/arm_controller/state", 1, urStateCallback);//simulator
  //ros::Subscriber sub = n.subscribe("/joint_states", 10, urStateCallback);//robot

  ros::Rate loop_rate(1);

  int count = 0;
  ros::Publisher ctrlPub = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/arm_controller/follow_joint_trajectory/goal", 1000); //simulator
  //ros::Publisher ctrlPub = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/follow_joint_trajectory/goal", 1000);//robot

  usleep(1000000);
  std::vector<std::vector<double>> trajectory={{0,0,0,0,0,0},{0.5,-0.7,0.7,0,0,0},{-0.5,-0.2,-0.3,0.4,0.5,0.6}};
  int pointNo=0;
  while (ros::ok())
  {

    ROS_INFO("count: %d", count);

    control_msgs::FollowJointTrajectoryActionGoal ctrlMsg;
    ctrlMsg.goal.trajectory.joint_names.push_back("shoulder_pan_joint");
    ctrlMsg.goal.trajectory.joint_names.push_back("shoulder_lift_joint");
    ctrlMsg.goal.trajectory.joint_names.push_back("elbow_joint");
    ctrlMsg.goal.trajectory.joint_names.push_back("wrist_1_joint");
    ctrlMsg.goal.trajectory.joint_names.push_back("wrist_2_joint");
    ctrlMsg.goal.trajectory.joint_names.push_back("wrist_3_joint");

    ctrlMsg.goal.trajectory.points.resize(1);
    ctrlMsg.goal.trajectory.points[0].positions.resize(6);
    //check if robot reached goal position
    bool goalReached = true;
    for (size_t jointNo=0; jointNo<6;jointNo++){
        if (fabs(currentPos[jointNo]-trajectory[pointNo][jointNo])>0.01){
            goalReached = false;
            break;
        }
    }
    //change reference position
    if (goalReached){
        pointNo++;
        if (pointNo>=trajectory.size())
            pointNo=0;
    }
    for (size_t jointNo=0; jointNo<6;jointNo++)
        ctrlMsg.goal.trajectory.points[0].positions[jointNo]=trajectory[pointNo][jointNo];
    // Velocities
    ctrlMsg.goal.trajectory.points[0].velocities.resize(6);
    for (size_t jointNo=0; jointNo<6;jointNo++)
        ctrlMsg.goal.trajectory.points[0].velocities[jointNo] = 0.0;
    ctrlMsg.goal.trajectory.points[0].time_from_start = ros::Duration(1.0);

    ctrlPub.publish(ctrlMsg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}
