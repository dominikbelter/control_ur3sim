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
   std::vector<std::vector<double>> trajectory={{0,0,0,0,0,0},{0.1,-0.7,0.7,0,0,0},{-0.5,-0.2,-0.3,0.4,0.5,0.6}};
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

//#include "ros/ros.h"

//// ROS
//#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <geometry_msgs/WrenchStamped.h>
//#include <message_filters/cache.h>
//#include <message_filters/subscriber.h>
//#include <tf/transform_listener.h>
//#include <control_msgs/JointTrajectoryControllerState.h> //read state of the arm
//#include <control_msgs/FollowJointTrajectoryAction.h> //set goal positions
//#include <control_msgs/FollowJointTrajectoryActionResult.h> // result
//#include <sensor_msgs/JointState.h>
//#include <std_srvs/Empty.h>

////MoveIt
//#include <moveit/robot_model_loader/robot_model_loader.h>
//#include <moveit/robot_state/robot_state.h>
//#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/planning_scene/planning_scene.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <moveit/planning_interface/planning_interface.h>
//#include <moveit/ompl_interface/ompl_interface.h>
//#include <moveit/kinematic_constraints/kinematic_constraint.h>
//#include <moveit/kinematic_constraints/utils.h>

//// pluginlib
//#include <pluginlib/class_loader.h>

//using namespace std;
//using namespace ros;
//using namespace tf;

//static const std::string PLANNING_GROUP = "manipulator";

//std::vector<double> currentJointsState;
//std::vector<double> referenceJointsVals;
//std::vector<double> currentPos = {1.54,-0.785,0.785,-1.54,-1.54,0};

//void urStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
//{
//   for (size_t jointNo=0; jointNo<6;jointNo++){
//       ROS_INFO("Joint pos[%d]: %f", (int)jointNo, msg->actual.positions[jointNo]);
//       currentPos[jointNo] = msg->actual.positions[jointNo];
//   }
//}

//int main(int argc, char **argv)
//{
//   ros::init(argc, argv, "ur3_control");

//   ros::NodeHandle n;

//   ros::Subscriber sub = n.subscribe("/arm_controller/state", 1, urStateCallback);//simulator
//   //ros::Subscriber sub = n.subscribe("/joint_states", 10, urStateCallback);//robot

//   //moveIt
//   robot_model_loader::RobotModelLoader robotModelLoader("robot_description");
//   /// kinematic model of the robot
//   robot_model::RobotModelPtr kinematic_model;
//   /// kinematic state
//   moveit::core::RobotStatePtr kinematic_state;
//   /// joints
//   const robot_state::JointModelGroup* joint_model_group;
//   /// ompl planner
//   planning_interface::PlannerManagerPtr planner_instance;
//   /// planning scene
//   planning_scene::PlanningScenePtr planningScene;
//   /// planning
//   moveit::planning_interface::MoveGroupInterface moveGroup(PLANNING_GROUP);

//   ros::Rate loop_rate(1);

//   int count = 0;

//   // load parameters
//   /// maxJointErrorTolerance
//   double maxJointErrorTolerance(0.025);

//   ros::Publisher ctrlPub = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/arm_controller/follow_joint_trajectory/goal", 1000); //simulator
//   //ros::Publisher ctrlPub = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/follow_joint_trajectory/goal", 1000);//robot

//   usleep(1000000);

//   control_msgs::FollowJointTrajectoryActionGoal ctrlMsg;
//   ctrlMsg.goal.trajectory.joint_names.push_back("shoulder_pan_joint");
//   ctrlMsg.goal.trajectory.joint_names.push_back("shoulder_lift_joint");
//   ctrlMsg.goal.trajectory.joint_names.push_back("elbow_joint");
//   ctrlMsg.goal.trajectory.joint_names.push_back("wrist_1_joint");
//   ctrlMsg.goal.trajectory.joint_names.push_back("wrist_2_joint");
//   ctrlMsg.goal.trajectory.joint_names.push_back("wrist_3_joint");

//   ctrlMsg.goal.trajectory.points.resize(1);
//   ctrlMsg.goal.trajectory.points[0].positions.resize(6);
//   std::vector<double> initConf = {1.54,-0.785,0.785,-1.54,-1.54,0};
//   for (size_t jointNo=0; jointNo<6;jointNo++)
//       ctrlMsg.goal.trajectory.points[0].positions[jointNo]=initConf[jointNo];
//   // Velocities
//   ctrlMsg.goal.trajectory.points[0].velocities.resize(6);
//   for (size_t jointNo=0; jointNo<6;jointNo++)
//       ctrlMsg.goal.trajectory.points[0].velocities[jointNo] = 0.0;
//   ctrlMsg.goal.trajectory.points[0].time_from_start = ros::Duration(1.0);

//   ctrlPub.publish(ctrlMsg);

//   kinematic_model = robotModelLoader.getModel();
//   ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

//   ///initialize MoveIt
//   kinematic_state.reset(new robot_state::RobotState(kinematic_model));
//   kinematic_state->setToDefaultValues();
//   joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP);

//   Eigen::VectorXd initConfiguration(6,1); initConfiguration << 1.54,-0.785,0.785,-1.54,-1.54,0;
//   kinematic_state->setJointGroupPositions(joint_model_group,initConfiguration);

//   /// initialize OMPL
//   planningScene.reset(new planning_scene::PlanningScene(kinematic_model));
//   //planning_scene::PlanningScene planning_scene(kinematic_model);

//   collision_detection::CollisionRequest collision_request;
//   collision_detection::CollisionResult collision_result;
//   planningScene->checkSelfCollision(collision_request, collision_result);
//   if (collision_result.collision)
//       ROS_INFO("Test 1: Current state is: collision");
//   else
//       ROS_INFO("Test 1: Current state is: no collision");

//   moveGroup.setPlannerId("RRTConnectkConfigDefault");

//   boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
//   std::string planner_plugin_name;

//   n.setParam("planning_plugin", "ompl_interface/OMPLPlanner");
//   if (!n.getParam("planning_plugin", planner_plugin_name))
//       ROS_FATAL_STREAM("Could not find planner plugin name");
//   try
//   {
//       planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
//                                       "moveit_core", "planning_interface::PlannerManager"));
//   }
//   catch (pluginlib::PluginlibException& ex)
//   {
//       ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
//   }
//   try
//   {
//       planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
//       if (!planner_instance->initialize(kinematic_model, n.getNamespace()))
//           ROS_FATAL_STREAM("Could not initialize planner instance");
//       ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
//   }
//   catch (pluginlib::PluginlibException& ex)
//   {
//       const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
//       std::stringstream ss;
//       for (std::size_t i = 0; i < classes.size(); ++i)
//           ss << classes[i] << " ";
//       ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
//                        << "Available plugins: " << ss.str());
//   }

//   usleep(1000000);
//   std::vector<Eigen::Affine3d> trajectory;
//   Eigen::Affine3d motion(Eigen::Affine3d::Identity());
//   motion(1,3)=-0.13;
//   trajectory.push_back(motion);
//   motion(0,3)=-0.13;
//   motion(1,3)=0.0;
//   trajectory.push_back(motion);
//   motion(0,3)=0.0;
//   motion(1,3)=0.13;
//   trajectory.push_back(motion);
//   motion(0,3)=0.13;
//   motion(1,3)=0.0;
//   trajectory.push_back(motion);
//   int pointNo=0;

//   std::vector<double> refJointValues(initConf);
//   while (ros::ok())
//   {
//       ROS_INFO("count: %d", count);

//       //check if robot reached goal position
//       bool goalReached = true;
//       for (size_t jointNo=0; jointNo<6;jointNo++){
//           if (fabs(currentPos[jointNo]-refJointValues[jointNo])>maxJointErrorTolerance){
//               goalReached = false;
//               break;
//           }
//       }
//       //change reference position
//       if (goalReached){
//           pointNo++;
//           if (pointNo>=trajectory.size())
//               pointNo=0;
//           const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("wrist_3_link");
//           Eigen::Affine3d newEndEffectorState = end_effector_state * trajectory[pointNo];
//           bool found_ik = kinematic_state->setFromIK(joint_model_group, newEndEffectorState, "wrist_3_link", 100, 0.1);
//           if (found_ik) {
//               // use IK results to execute motion
//               // should be moveit::planning_interface::MoveGroupInterface::Plan my_plan
//               // remeber to use additional time and ros::AsyncSpinner if 'moveGroup.plan(my_plan)' locks main loop
//               kinematic_state->copyJointGroupPositions(joint_model_group, refJointValues);
//               for (size_t jointNo=0; jointNo<6;jointNo++)
//                   ctrlMsg.goal.trajectory.points[0].positions[jointNo]=refJointValues[jointNo];
//               ctrlPub.publish(ctrlMsg);
//           }
//           else {
//               ROS_INFO("Did not find IK solution");
//           }
//       }
//       ros::spinOnce();

//       loop_rate.sleep();
//       ++count;
//   }
//   return 0;
//}
