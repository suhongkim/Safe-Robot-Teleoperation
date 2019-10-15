/**
 * @file safe_teleop.cpp
 * @brief Safe teleoperation library implementation
 * Created by rakesh on 28/09/18.
 */
#include <limits>
#include <safe_teleop/safe_teleop.h>

namespace safe_teleop
{

SafeTeleop::SafeTeleop() :
  is_shutdown_(false),
  max_cmd_vel_age_(1.0),
  max_linear_vel_(1.0),
  max_angular_vel_(1.0),
  linear_vel_increment_(0.05),
  angular_vel_increment_(0.05),
  laser_safety_check_angle_(0.25),
  min_safety_impact_time_(0.5),
  min_safety_distance_(0.5),
  linear_vel_(0.0),
  angular_vel_(0.0),
  last_command_timestamp_(0.0)
{
  ros::NodeHandle global_nh;
  cmd_vel_pub_ = global_nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
  // The subscriber callback is set to the laserScanCallback method of the instantiated object of this class
  laser_scan_sub_ = global_nh.subscribe("scan", 5, &SafeTeleop::laserScanCallback, this);

  run_thread_ = boost::thread(&SafeTeleop::run, this);
  displayCurrentSpeeds();
}

SafeTeleop::~SafeTeleop()
{
  shutdown();
  // wait for the run thread to terminate
  run_thread_.join();

  geometry_msgs::Twist zero_cmd_vel;
  zero_cmd_vel.linear.x = 0;
  zero_cmd_vel.angular.z = 0;
  cmd_vel_pub_.publish(zero_cmd_vel);
}

void SafeTeleop::run()
{
  ros::Rate r(10);
  while (ros::ok() && !is_shutdown_)
  {
    auto current_timestamp = ros::Time::now().toSec();

    auto last_cmd_vel_age = current_timestamp - last_command_timestamp_;

    if (last_cmd_vel_age > max_cmd_vel_age_)
    {
      // when no command is received for 1 second, set both vels to zero 
      linear_vel_=0; 
      angular_vel_=0; 
    }
    else
    {
      // check safety 
      auto is_safe = checkSafety(static_cast<double>(linear_vel_));
      // if not safe, set linear vel to zero and send warning to user
      if (!is_safe)
      {
        linear_vel_=0;
        ROS_WARN_THROTTLE(1.0, "Not Safe!!! Emergency Stop!! \r");
      }
    }

    // publish cmd_vel every 10 hz 
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear_vel_;
    cmd_vel.angular.z = angular_vel_;
    cmd_vel_pub_.publish(cmd_vel);

    r.sleep();
  }
}

void SafeTeleop::moveForward()
{
  if (linear_speed_ < 0) //initial pose: backward
  {
    linear_vel_ = (-1) * (double)linear_speed_;
  }
  else //initial pose: forward
  {
    linear_vel_ = (double)linear_speed_;
  }
  angular_vel_ = 0; // No arc movement 
  last_command_timestamp_ = ros::Time::now().toSec();
}

void SafeTeleop::moveBackward()
{
  if (linear_speed_ > 0) //initial pose: forward
  {
    linear_vel_ = (-1) * (double)linear_speed_;
  }
  else //initial pose: backward
  {
    linear_vel_ = (double)linear_speed_;
  }
  angular_vel_ = 0; // No arc movement 
  last_command_timestamp_ = ros::Time::now().toSec();
}

void SafeTeleop::rotateClockwise()
{
  if (angular_speed_ < 0) //initial pose: counterclockwise
  {
    angular_vel_ = (-1) * (double)angular_speed_;
  }
  else //initial pose: clockwise
  {
    angular_vel_ = (double)angular_speed_;
  }
  linear_vel_ = 0; // No arc movement 
  last_command_timestamp_ = ros::Time::now().toSec();
}

void SafeTeleop::rotateCounterClockwise()
{
  if (angular_speed_ > 0) //initial pose: clockwise
  {
    angular_vel_ = (-1) * (double)angular_speed_;
  }
  else //initial pose: counterclockwise
  {
    angular_vel_ = (double)angular_speed_;
  }
  linear_vel_ = 0; // No arc movement 
  last_command_timestamp_ = ros::Time::now().toSec();
}

void SafeTeleop::stop()
{
  // Initialize both spped to zero 
  linear_speed_ = 0; 
  angular_speed_ = 0; 
  linear_vel_ = (double)linear_speed_; 
  angular_vel_ = (double)angular_speed_; 
  last_command_timestamp_ = ros::Time::now().toSec();
}


void SafeTeleop::increaseLinearSpeed()
{
  if (linear_speed_ < max_linear_vel_) // check max spped 
  {
    linear_speed_ = linear_speed_ + linear_vel_increment_;
  }
  else 
  {
    linear_speed_ = max_linear_vel_; // set max
  }
  last_command_timestamp_ = ros::Time::now().toSec();
  displayCurrentSpeeds();
}

void SafeTeleop::decreaseLinearSpeed()
{
  if (linear_speed_ >= linear_vel_increment_) // check min spped 
  {
    linear_speed_ = linear_speed_ - linear_vel_increment_;
  }
  else 
  {
    linear_speed_ = (double)0; // set zero 
  }
  last_command_timestamp_ = ros::Time::now().toSec();
  displayCurrentSpeeds();
}

void SafeTeleop::increaseAngularSpeed()
{
  if (angular_speed_ < max_angular_vel_) // check max spped 
  {
    angular_speed_ = angular_speed_ + angular_vel_increment_;
  }
  else 
  {
    linear_speed_ = max_linear_vel_; // set max
  }
  last_command_timestamp_ = ros::Time::now().toSec();
  displayCurrentSpeeds();
}

void SafeTeleop::decreaseAngularSpeed()
{
  if (angular_speed_ >= angular_vel_increment_) // check min spped 
  {
    angular_speed_ = angular_speed_ - angular_vel_increment_;
  }
  else 
  {
    angular_speed_ = (double)0;  // set zero 
  }
  last_command_timestamp_ = ros::Time::now().toSec();
  displayCurrentSpeeds();
}

// For Simultation 

bool SafeTeleop::checkSafety(double linear_vel)
{
  auto laser_scan = getLaserScan();
  
  // angle_increment : 0.049474
  // range_min: 0   # distance min
  // range_max: 30  # distance max 
  // angle_min: -3.141593  = -180 degree
  // angle_max: 3.141593   = 180 degree
  // total size of ranges: 128

  // check if the scan measurements are recieved or not 
  if (laser_scan.ranges.size() == 0) 
  {
    return false; 
  }

  // check front/back obstacles 
  if (linear_vel >= 0) // Forward 
  {
    for (int i = 58; i < 70; i++) // +15 ~ 0 ~ -15 degrees
    {
      if (laser_scan.ranges[i] < min_safety_distance_)
        return false; 
    }
  }
  else // backword 
  {
    for (int i = 0; i < 6; i++) // 165 ~ 180 degrees
    {
      if (laser_scan.ranges[i] < min_safety_distance_)
        return false; 
    }
    for (int i = 122; i < 128; i++) // 180 ~ 195 degrees
    {
      if (laser_scan.ranges[i] < min_safety_distance_)
        return false; 
    }
  }

  return true;
}

//// For Turtlebot 
/*
bool SafeTeleop::checkSafety(double linear_vel)
{
  auto laser_scan = getLaserScan();
  
  // angle_increment : 0.049474
  // range_min: 0   # distance min
  // range_max: 3.5  # distance max 
  // angle_min: 0.0   = 0 degree
  // angle_max: 6.26573181152    = 180 degree
  // total size of ranges: 360

  // check if the scan measurements are recieved or not 
  if (laser_scan.ranges.size() == 0) 
  {
    return false; 
  }

  // check front/back obstacles 
  if (linear_vel >= 0) // Forward 
  {
    for (int i = 0; i < 15; i++) // 0 ~ +15  degrees
    {
      if (laser_scan.ranges[i] > 0.0 && laser_scan.ranges[i] < min_safety_distance_){
        ROS_INFO("front laser_scan_front: %f\r", laser_scan.ranges[i]);
        return false; 
      }
    }
    for (int i = 345; i < 360; i++) // -15 ~ 0  degrees
    {
      if (laser_scan.ranges[i] > 0.0 && laser_scan.ranges[i] < min_safety_distance_){
        ROS_INFO("front laser_scan_front: %f\r", laser_scan.ranges[i]);
        return false; 
      }
    }
  }
  else // backword 
  {
    for (int i = 165; i < 180; i++) // 165 ~ 180 degrees
    {
      if (laser_scan.ranges[i] > 0.0 && laser_scan.ranges[i] < min_safety_distance_)
      {
        ROS_INFO("rear laser_scan_front: %f\r", laser_scan.ranges[i]);
        return false; 
      }
    }
    for (int i = 180; i < 195; i++) // 180 ~ 195 degrees
    {
      if (laser_scan.ranges[i] > 0.0 && laser_scan.ranges[i] < min_safety_distance_)
      {
        ROS_INFO("rear laser_scan_front: %f\r", laser_scan.ranges[i]);
        return false; 

      }
    }
  }

  return true;
}
*/


} // namespace safe_teleop_node


