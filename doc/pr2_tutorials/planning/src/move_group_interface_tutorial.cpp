#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <string>
#include <sys/time.h>
#include "ompl/util/Time.h"
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

typedef move_group_interface::MoveGroup C_moveGroup;
typedef moveit::planning_interface::MoveGroup::Plan S_Plan;
typedef moveit::planning_interface::PlanningSceneInterface S_PlanningSceneInterface;

void IK_action_one(C_moveGroup &group, double position_action[])
{
  geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id    = group.getPoseReferenceFrame();
    target_pose.header.stamp       = ros::Time::now();  
    target_pose.pose.position.x    = position_action[0];
    target_pose.pose.position.y    = position_action[1];
    target_pose.pose.position.z    = position_action[2];  
    target_pose.pose.orientation.x = position_action[3];
    target_pose.pose.orientation.y = position_action[4];
    target_pose.pose.orientation.z = position_action[5];
    target_pose.pose.orientation.w = position_action[6];

  group.setPoseTarget(target_pose, group.getEndEffectorLink()); 
  group.setStartStateToCurrentState() ;
  group.move();
  usleep(500000);
}

int discretize_circle(double circleCenter[], std::vector<std::vector<double> > &V_pose_value)
{ 
  std::vector<double> pose_value(7);
  pose_value[1] = circleCenter[1];
  pose_value[3] = circleCenter[3];
  pose_value[4] = circleCenter[4];
  pose_value[5] = circleCenter[5];
  pose_value[6] = circleCenter[6];

  double y_center = circleCenter[0];
  double z_center = circleCenter[2];    
  double angle= 0;
  double radius = 0.11;    
  double angle_resolution = 5;                /* 两个采样点间的角度值，减小可提高轨迹规划成功率 */
    double d_angle = angle_resolution*3.14/180; /* 两个采样点间的弧度值 */

    //  采样圆上的点
    for (int i= 0; i< (360/angle_resolution); i++)
    {
      angle+= d_angle; 
    pose_value[0] = y_center + radius*sin(angle);
        pose_value[2] = z_center + radius*cos(angle);        
        V_pose_value.push_back(pose_value);   
      printf("%f %f %f %f %f %f %f \n",pose_value[0],pose_value[1],pose_value[2],pose_value[3],pose_value[4],pose_value[5],pose_value[6]);
    } 
  return 0;
}

int discretize_circle_a(double circleCenter[], std::vector<std::vector<double> > &W_pose_value)
{ 
  std::vector<double> pose_value(7);
  pose_value[1] = circleCenter[1];
  pose_value[3] = circleCenter[3];
  pose_value[4] = circleCenter[4];
  pose_value[5] = circleCenter[5];
  pose_value[6] = circleCenter[6];

  double y_center = circleCenter[0];
  double z_center = circleCenter[2];    
  double angle_a= -80;
  double radius = 0.11;    
  double angle_resolution = 5;                /* 两个采样点间的角度值，减小可提高轨迹规划成功率 */
    double d_angle_a = angle_resolution*3.14/180; /* 两个采样点间的弧度值 */

    //  采样圆上的点
    for (int i= 0; i< (90/angle_resolution); i++)
    {
      angle_a-= d_angle_a; 
    pose_value[0] = y_center + radius*sin(angle_a);
        pose_value[2] = z_center + radius*cos(angle_a);        
        W_pose_value.push_back(pose_value);   
      printf("%f %f %f %f %f %f %f \n",pose_value[0],pose_value[1],pose_value[2],pose_value[3],pose_value[4],pose_value[5],pose_value[6]);
    } 
  return 0;
}

int draw_circle(C_moveGroup &group, double circleCenter[])
{
  int i, j, len;
  S_Plan plan; 
  std::vector<std::vector<double> > V_pose_value;
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose ee_point_goal_tmp;

  /* 采样圆上的点 */
  discretize_circle(circleCenter, V_pose_value);
  for(i=0; i<V_pose_value.size(); i++)
  {
    ee_point_goal_tmp.position.x    = V_pose_value[i][0];
    ee_point_goal_tmp.position.y    = V_pose_value[i][1];
    ee_point_goal_tmp.position.z    = V_pose_value[i][2];
    ee_point_goal_tmp.orientation.x = V_pose_value[i][3];
    ee_point_goal_tmp.orientation.y = V_pose_value[i][4];
    ee_point_goal_tmp.orientation.z = V_pose_value[i][5];
    ee_point_goal_tmp.orientation.w = V_pose_value[i][6];
    waypoints.push_back(ee_point_goal_tmp);
  }
  
  /* 将end_effector移动到0度的地方即圆心正上方的圆上点 */
  double position_action[7];
  for(i=0; i<sizeof(position_action)/sizeof(position_action[0]); i++)
  {
    position_action[i] = circleCenter[i];
    if(2 == i)
      position_action[i] += 0.1;
  } 
  IK_action_one(group, position_action);

  /* 添加画圆的圈数 */
  for(j=0; j<0; j++)
  {
    for(i=0; i<len; i++)
    {
      ee_point_goal_tmp = waypoints[i];
      waypoints.push_back(ee_point_goal_tmp);
    }
  }

  double eef_resolution = 0.01;
    double jump_threshold  = 0.0;
    moveit_msgs::RobotTrajectory trajectory;
  i = 0;
  while(i<100)
  {/* 100次规划直到轨迹规划成功 */
    i++;
    double fraction =group.computeCartesianPath(waypoints, eef_resolution,jump_threshold, trajectory);   
    printf("T  ---  fraction[%f]  waypoints.size(%lu)\n", fraction, waypoints.size());
    if(1.0 == fraction)
      break;
  } 
  plan.trajectory_ = trajectory;
  group.execute(plan);
  usleep(500000);    
  return 0;
}

int draw_circle_a(C_moveGroup &group, double circleCenter[])
{
  int i, j, len;
  S_Plan plan; 
  std::vector<std::vector<double> > W_pose_value;
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose ee_point_goal_tmp_a;

  /* 采样圆上的点 */
  discretize_circle_a(circleCenter, W_pose_value);
  for(i=0; i<W_pose_value.size(); i++)
  {
    ee_point_goal_tmp_a.position.x    = W_pose_value[i][0];
    ee_point_goal_tmp_a.position.y    = W_pose_value[i][1];
    ee_point_goal_tmp_a.position.z    = W_pose_value[i][2];
    ee_point_goal_tmp_a.orientation.x = W_pose_value[i][3];
    ee_point_goal_tmp_a.orientation.y = W_pose_value[i][4];
    ee_point_goal_tmp_a.orientation.z = W_pose_value[i][5];
    ee_point_goal_tmp_a.orientation.w = W_pose_value[i][6];
    waypoints.push_back(ee_point_goal_tmp_a);
  }
  
  /* 将end_effector移动到0度的地方即圆心正上方的圆上点 */
 

  /* 添加画圆的圈数 */
  for(j=0; j<0; j++)
  {
    for(i=0; i<len; i++)
    {
      ee_point_goal_tmp_a = waypoints[i];
      waypoints.push_back(ee_point_goal_tmp_a);
    }
  }

  double eef_resolution = 0.01;
    double jump_threshold  = 0.0;
    moveit_msgs::RobotTrajectory trajectory;
  i = 0;
  while(i<100)
  {/* 100次规划直到轨迹规划成功 */
    i++;
    double fraction =group.computeCartesianPath(waypoints, eef_resolution,jump_threshold, trajectory);   
    printf("T  ---  fraction[%f]  waypoints.size(%lu)\n", fraction, waypoints.size());
    if(1.0 == fraction)
      break;
  } 
  plan.trajectory_ = trajectory;
  group.execute(plan);
  usleep(500000);    
  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  /*double circleCenter[] = {-0.0740419807012,-0.827934028816,0.071384019565,-0.442415648175,0.529224646862,-0.517958940652,0.505873702817};
*/
  double circleCenter[] = {-0.0740419807012,-0.827934028816,0.171384019565,-0.442415648175,0.529224646862,-0.517958940652,0.505873702817};
  C_moveGroup manipulator("manipulator");
  manipulator.setPlannerId("RRTConnectkConfigDefault");
  manipulator.setNumPlanningAttempts(10);
  manipulator.allowReplanning(true); 
  
  
  draw_circle(manipulator, circleCenter);
  /*ros::Duration(2.5).sleep();
  draw_circle_a(manipulator, circleCenter);
  ros::Duration(3.5).sleep();
  draw_circle(manipulator, circleCenter);
  ros::Duration(2.5).sleep();
  draw_circle_a(manipulator, circleCenter);
  ros::Duration(6).sleep();
  draw_circle(manipulator, circleCenter);
  ros::Duration(2.5).sleep();
  draw_circle_a(manipulator, circleCenter);
  ros::Duration(3.5).sleep();
  draw_circle(manipulator, circleCenter);
  ros::Duration(2.5).sleep();
  draw_circle_a(manipulator, circleCenter);
*/

  return 0;
}