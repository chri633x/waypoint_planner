#include "ros/ros.h"
using namespace std;
#include <math.h>
#include <time.h>
#define _USE_MATH_DEFINES
#include <cmath>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

class SubscribeAndPublish {
  public:
    SubscribeAndPublish()  {
      pub_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel" ,1);
      sub_point = n.subscribe("clicked_point", 10, &SubscribeAndPublish::callback_point, this);
      sub_goal = n.subscribe("move_base_simple/goal", 10, &SubscribeAndPublish::callback_goal, this);
      sub_scan = n.subscribe("scan", 1, &SubscribeAndPublish::checkForObstacle, this);
    }

  private:
    ros::NodeHandle n;
    ros::Publisher pub_vel;
    ros::Subscriber sub_point;
    ros::Subscriber sub_goal;
    ros::Subscriber sub_scan;
    tf::TransformListener listener;

    struct Point{
      float x;
      float y;
    };
    vector<Point> points; //vector storing all waypoints and goal position

    //parameters
    float ang_vel_max = 2.84; //upper limit for angular velocity for Turtlebot3
    float lin_vel_max = 0.22; //upper limit for linear velocity for Turtlebot3
  
    float k_p_lin = lin_vel_max/ang_vel_max; //linear velocity gain
    float k_p_ang = M_PI/ang_vel_max; //angular velocity gain
    bool obstacle = false; //boolean variable that keeps track of if an obstacle is in front of the robot
    int view = 40; //the robot's sight in degrees for obstacle detection (must be an even number!). 40 degrees means 20 degrees of sight to each side
    float safety_dist = 0.5; //safety distance to obstacles
    float goal_orientation_tolerance = 0.087; //the orientation at goal must be within 5 degrees
    float goal_distance_tolerance = 0.05; //the distance to goal should be below 5 cm
    float waypoint_distance_tolerance = 0.10; //the distance to waypoints should be below 10 cm

    tf::StampedTransform get_transform(); //transform from "base_footprint" to "map"
    void callback_point(const geometry_msgs::PointStamped::ConstPtr& msg); //receives points from "publish point" in rviz
    void callback_goal(const geometry_msgs::PoseStamped::ConstPtr& msg); //receives goal position from "2D Nav Goal" in rviz
    void checkForObstacle(const sensor_msgs::LaserScan::ConstPtr& msg); //receives data from laserscanner and checks for obstacle
    void sendCommands(float goal_x, float goal_y); //sends linear and angular velocities to the turtlebot
    float correctOrientationAtGoal(float goal_w, float goal_z); //corrects the orientation at goal position. This function is quite similar to sendCommands() function
    float getDist(float goal_x, float goal_y); //returns distance from current pose to goal pose (last element in point vector)
    void stop(); //makes the robot stop at goal position
};

tf::StampedTransform SubscribeAndPublish::get_transform() { //transform from "base_footprint" to "map"

     tf::StampedTransform transform;
     try{
       listener.lookupTransform("map","base_footprint",
                                ros::Time(0), transform);
     }
     catch (tf::TransformException ex){
       ROS_ERROR("%s",ex.what());
       ros::Duration(1.0).sleep();
     }

     return transform;
}

void SubscribeAndPublish::callback_point(const geometry_msgs::PointStamped::ConstPtr& msg) { //receives points from "publish point" in rviz

     float x_received = msg->point.x;
     float y_received = msg->point.y;
     //ROS_INFO("received target: x: %f, y: %f", x_received, y_received);

     points.push_back({x_received, y_received});
}

void SubscribeAndPublish::callback_goal(const geometry_msgs::PoseStamped::ConstPtr& msg) {

     float goal_x = msg->pose.position.x;
     float goal_y = msg->pose.position.y;
     float goal_w = msg->pose.orientation.w;
     float goal_z = msg->pose.orientation.z;

     points.push_back({goal_x, goal_y});

     for (int i=0; i<points.size(); i++){

         ROS_INFO("Moving towards point: %i", i+1);

         while(getDist(points[i].x, points[i].y)>waypoint_distance_tolerance){ //The next waypoint goal should not be set before the robot is closer than 10 cm to current waypoint goal

            ros::spinOnce(); //update callbacks (for scan information)

            if(!obstacle){ //if no obstacle - send command
              sendCommands(points[i].x, points[i].y);
              usleep(100000); //10 hz
            }
            else{
              stop(); //stop until obstacle is gone
              usleep(100000); //10 hz
            }
         };

         if (i==points.size()-1){ //stop at the last element in vector (goal position)
             while(getDist(points[i].x, points[i].y)>goal_distance_tolerance){ //The robot must be within a distance of 5 cm to the goal
               sendCommands(points[i].x, points[i].y);
               usleep(100000); //10 hz
             };
             ROS_INFO("Correcting orientation at goal");

             while(abs(correctOrientationAtGoal(goal_w, goal_z))>goal_orientation_tolerance){ //the orientation at goal must be within 5 degrees
               usleep(100000); //10 hz
             };

             stop();
             ROS_INFO("Goal reached");
         }
     }
     points.clear(); //clear vector since goal is reached
}

void SubscribeAndPublish::checkForObstacle(const sensor_msgs::LaserScan::ConstPtr& msg){ //receives data from laserscanner and checks for obstacle

  vector <float> temp_vector; //temporary vector for storing distances

  for(int i=0; i<view/2; i++){ //get angles from [0;20] degrees (if view is 40)
    temp_vector.push_back(msg->ranges[i]);
  }
  for(int i=360-(view/2); i<360; i++){//get angles from [-20;0] degrees (if view is 40)
    temp_vector.push_back(msg->ranges[i]);
  }

  for(int i=0; i<view; i++){
    if(temp_vector[i]>0.01 && temp_vector[i]<safety_dist){
      obstacle = true;
      return;
    }
  }
  obstacle = false;
}

void SubscribeAndPublish::sendCommands(float goal_x, float goal_y){ //sends linear and angular velocities to the turtlebot
  geometry_msgs::Twist msg;
  tf::StampedTransform transform = get_transform();

  //get current position
  float current_x = transform.getOrigin().x();
  float current_y = transform.getOrigin().y();

  //get current rotation
  float current_w = transform.getRotation().getW();
  float current_z = transform.getRotation().getZ();

  //velocity variables
  float vel_lin;
  float vel_ang;

  float goal_yaw = atan2 (goal_y-current_y, goal_x-current_x); //desired orientation in RPY radians
  float current_yaw = atan2(2.0*current_w*current_z, current_w*current_w - current_z*current_z);  //convert current orientation in quaternions to RPY (only Yaw is needed)

  //avoid weird rotation when current_yaw goes from Pi to -Pi
  if (current_yaw<0){
    current_yaw = 2*M_PI + current_yaw;
  }

  float diff = current_yaw-goal_yaw; //orientation error

  //the bigger orientation error, the more angular speed
  if (diff<=0){ //turn left
    vel_ang = abs(diff) * k_p_ang;
  }
  else if (diff >= M_PI){ //turn left
    vel_ang = (2*M_PI-diff) * k_p_ang;
  }
  else{ //turn right
    vel_ang = -diff * k_p_ang;
  }

  vel_lin = abs((ang_vel_max-abs(vel_ang)))*k_p_lin; //the less angular velocity, the more linear velocity (P-controller)

  msg.linear.x = vel_lin;
  msg.angular.z = vel_ang;

  pub_vel.publish(msg); //publish velocities
}

float SubscribeAndPublish::correctOrientationAtGoal(float goal_w, float goal_z){ //corrects the orientation at goal position. This function is quite similar to sendCommands() function
  geometry_msgs::Twist msg;
  tf::StampedTransform transform = get_transform();

  //get current rotation
  float current_w = transform.getRotation().getW();
  float current_z = transform.getRotation().getZ();

  float vel_ang;       //velocity variable

  float current_yaw = atan2(2.0*current_w*current_z, current_w*current_w - current_z*current_z);  //convert current orientation in quaternions to RPY (only Yaw is needed)
  float goal_yaw = atan2(2.0*goal_w*goal_z, goal_w*goal_w - goal_z*goal_z);  //convert goal orientation in quaternions to RPY (only Yaw is needed)

  //avoid weird rotation when current_yaw goes from Pi to -Pi
  if (current_yaw<0){
    current_yaw = 2*M_PI + current_yaw;
  }

  float diff = current_yaw-goal_yaw; //orientation error

  //the bigger orientation error, the more angular speed
  if (diff<=0){ //turn left
    vel_ang = abs(diff) * k_p_ang;
  }
  else if (diff >= M_PI){ //turn left
    vel_ang = (2*M_PI-diff) * k_p_ang;
  }
  else{ //turn right
    vel_ang = -diff * k_p_ang;
  }

  msg.linear.x = 0.0;
  msg.angular.z = vel_ang;

  pub_vel.publish(msg); //publish velocities

  if (diff >= M_PI){
    return 2*M_PI-diff;
  }

  else {
    return diff;
  }
}

float SubscribeAndPublish::getDist(float goal_x, float goal_y){ //returns distance from current pose to goal pose (last element in point vector)

  tf::StampedTransform transform = get_transform();

  //get current pose and orientation
  float current_x = transform.getOrigin().x();
  float current_y = transform.getOrigin().y();
  float current_w = transform.getRotation().getW();
  float current_z = transform.getRotation().getZ();

  float dist = sqrt((goal_y-current_y)*(goal_y-current_y) + (goal_x-current_x)*(goal_x-current_x));

  //ROS_INFO("Distance: %f", dist);

  return dist; //return distance from current location to waypoint
}

void SubscribeAndPublish::stop(){ //makes the robot stop at goal position
  geometry_msgs::Twist msg;

  msg.linear.x = 0; //0.22 maximum
  msg.angular.z = 0; //2.84 maximum

  pub_vel.publish(msg);
}

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "waypoint_planner");

  SubscribeAndPublish SAPObject;

  ros::Rate loop_rate(10);
  ros::spin();
  loop_rate.sleep();

  return 0;
}
