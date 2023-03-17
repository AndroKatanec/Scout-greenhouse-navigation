/*
 * Authors: Nick Hillier and Fred Pauling (CSIRO, 2011)
 * 
 * Based on the sicklms.cpp from the sicktoolbox_wrapper ROS package
 * and the sample code from the sicktoolbox manual.
 * 
 * Released under BSD license.
 */ 

#include <iostream>
#include <sstream>
#include <sicktoolbox/SickNAV350.hh>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h" 
#include <deque>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sicktoolbox_wrapper/navtimestamp.h>

#define DEG2RAD(x) ((x)*M_PI/180.)

const double TRANSFORM_TIMEOUT = 20.0f;
const double POLLING_DURATION = 0.05f;
const std::string ODOM_TOPIC = "odom";

namespace OperatingModes
{
  enum OperatingMode
  {
    POWERDOWN = 0,
    STANDBY = 1,
    MAPPING = 2,
    LANDMARK = 3,
    NAVIGATION = 4,
  };
}
typedef OperatingModes::OperatingMode OperatingMode;

using namespace std;
using namespace SickToolbox;

// TODO: refactor these functions into a common util lib (similar to code in sicklms.cpp)
void publish_scan(ros::Publisher *pub, double *range_values,
                  uint32_t n_range_values, int *intensity_values,
                  uint32_t n_intensity_values, ros::Time start,
                  double scan_time, bool inverted, float angle_min,
                  float angle_max, std::string frame_id,
                  unsigned int sector_start_timestamp,ros::Publisher *pub1)
{
  static int scan_count = 0;
  sensor_msgs::LaserScan scan_msg;
  scan_msg.header.frame_id = frame_id;
  scan_count++;
  if (inverted) { // assumes scan window at the bottom
    scan_msg.angle_min = angle_max;
    scan_msg.angle_max = angle_min;
  } else {
    scan_msg.angle_min = angle_min;
    scan_msg.angle_max = angle_max;
  }
  scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(n_range_values-1);
  scan_msg.scan_time = 0.125;//scan_time;
  scan_msg.time_increment = scan_msg.scan_time/*scan_time*/ / n_range_values;
  scan_msg.range_min = 0.1;
  scan_msg.range_max = 250.;
  scan_msg.ranges.resize(n_range_values);
  scan_msg.header.stamp = start;
  for (size_t i = 0; i < n_range_values; i++) {
    scan_msg.ranges[i] = (float)range_values[i]/1000;
  }
  scan_msg.intensities.resize(n_intensity_values);
  for (size_t i = 0; i < n_intensity_values; i++) {
    scan_msg.intensities[i] = (float)intensity_values[i];
  }
  pub->publish(scan_msg);
  sicktoolbox_wrapper::navtimestamp st;
  st.stamp=start;
  st.navtimestamp=sector_start_timestamp;
  pub1->publish(st);
}

// A complimentary filter to get a (much) better time estimate, does not
// calibrate out constant network latency delays, but does get rid of 
// timming jitter to get better timing estimates than the 
// communicated clock resolution (which is only 1ms)
class smoothtime { 
protected:
  ros::Time smoothtime_prev, smoothed_timestamp;
  double time_smoothing_factor;
  double error_threshold;
public:
  //!
  smoothtime(){
    time_smoothing_factor = 0.95; /// slowly skew the clocks into sync
    error_threshold = .50; /// 50% jitter is acceptable for , discard data otherwise.
  }
  //! Between 0 and 1, bigger is smoother
  void set_smoothing_factor(double smoothing_factor){
    time_smoothing_factor = smoothing_factor;
  }
  //! Between 0 and 1, threshold on jitter acceptability, higher accepts more jitter before discarding
  void set_error_threshold(double err_threshold){
    error_threshold = err_threshold;
  }
  ros::Time smooth_timestamp(ros::Time recv_timestamp, ros::Duration expctd_dur) {
    if (smoothtime_prev.is_zero() == true) {
      smoothed_timestamp = recv_timestamp;
    } else {
      smoothed_timestamp = smoothtime_prev + expctd_dur;
      double err = (recv_timestamp - smoothed_timestamp).toSec();
      double time_error_threshold = expctd_dur.toSec() * error_threshold;
      if ((time_smoothing_factor > 0) && (fabs(err) < time_error_threshold)){
        ros::Duration correction = ros::Duration(err * (1 - time_smoothing_factor));
        smoothed_timestamp += correction;
      } else {
        // error too high, or smoothing disabled - set smoothtime to last timestamp
        smoothed_timestamp = recv_timestamp;
      }
    }
    smoothtime_prev = smoothed_timestamp;
    return smoothed_timestamp;
  }
};

void createOdometryMessage(const ros::Duration& time_elapsed, const tf::Transform& prev_transform,
                           const tf::Transform& current_transform, nav_msgs::Odometry& odom_msg)
{
  double dt = time_elapsed.toSec();
  double dx = (current_transform.getOrigin().getX() - prev_transform.getOrigin().getX())/dt;
  double dy = (current_transform.getOrigin().getY() - prev_transform.getOrigin().getY())/dt;
  double dr = (tf::getYaw(current_transform.getRotation()) - tf::getYaw(prev_transform.getRotation()))/dt;

  // setting position
  odom_msg.pose.pose.position.x = current_transform.getOrigin().getX();
  odom_msg.pose.pose.position.y = current_transform.getOrigin().getY();
  odom_msg.pose.pose.position.z = 0.0f;
  odom_msg.pose.covariance.assign(0.0f);
  tf::quaternionTFToMsg(current_transform.getRotation(),odom_msg.pose.pose.orientation);

  // set velocity
  odom_msg.twist.twist.linear.x = dx;
  odom_msg.twist.twist.linear.y = dy;
  odom_msg.twist.twist.linear.z = 0.0f;
  odom_msg.twist.twist.angular.x = 0.0f;
  odom_msg.twist.twist.angular.y = 0.0f;
  odom_msg.twist.twist.angular.z = dr;
  odom_msg.twist.covariance.assign(0.0f);

}

class averager {
protected:
  std::deque<double> deq;
  unsigned int max_len;
  double sum;
public:
  averager(int max_len = 50){
    this->max_len = max_len;
  }
  void add_new(double data) {
    deq.push_back(data);
    sum += data;
    if (deq.size() > max_len) {
      sum -= deq.front();
      deq.pop_front();
    }
  }
  double get_mean() {
    return sum/deq.size();
  }
};

void PublishReflectorTransform(std::vector<double> x,std::vector<double> y,double th,std::vector<tf::TransformBroadcaster> odom_broadcaster,std::string frame_id,std::string child_frame_id)
{
  //printf("\npos %.2f %.2f %.2f\n",x,y,th);
  tf::Transform b_transforms;
  tf::Quaternion tfquat=tf::createQuaternionFromYaw(th);//th should be 0? no orientation information on reflectors?

  std::ostringstream childframes;
  for (int i=0;i<x.size();i++)
  {
    childframes << child_frame_id << i;
    b_transforms.setRotation(tfquat);
    b_transforms.setOrigin(tf::Vector3(-x[i] / 1000, -y[i] / 1000, 0.0));
    ROS_DEBUG_STREAM("Reflector "<<i<<" x pos: "<<-x[i]<<" y pos: "<<-y[i]<<" with frame: "<<childframes.str());

    //send the transform
    odom_broadcaster[i].sendTransform(tf::StampedTransform(b_transforms, ros::Time::now(),
                                                           frame_id, childframes.str()));
    childframes.str(std::string());
    childframes.clear();
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "sicknav350");
  int op_mode, port, wait, mask;
  std::string ipaddress;
  std::string frame_id;
  std::string scan;
  bool inverted, do_mapping;
  bool publish_odom;
  int sick_motor_speed = 8;//10; // Hz
  double sick_step_angle = 1.5;//0.5;//0.25; // deg (0.125 = no gaps between spots)
  double active_sector_start_angle = 0;
  double active_sector_stop_angle = 360;//269.75;
  double smoothing_factor, error_threshold;
  std::string sick_frame_id; 
  std::string target_frame_id; // the frame to be publish relative to frame_id
  std::string scan_frame_id;   // The frame reported in the scan message.
                               // Decoupling this from target_frame_id allows us to
                               // display nav localization and ROS localization
                               // simultaneously in RViz.
  std::string mobile_base_frame_id = "";
  std::string reflector_frame_id,reflector_child_frame_id;
  tf::StampedTransform sickn350_to_target_tf;
  tf::StampedTransform target_to_mobile_base_tf;

  ros::NodeHandle nh;
  ros::NodeHandle nh_ns("~");
  nh_ns.param<std::string>("scan", scan, "scan");
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>(scan, 1);
  ros::Publisher scan_pub1 = nh.advertise<sicktoolbox_wrapper::navtimestamp>("navtimestamp", 1);
  ros::Publisher odom_pub;

  nh_ns.param("mode", op_mode, 4);
  nh_ns.param("port", port, DEFAULT_SICK_TCP_PORT);
  nh_ns.param("ipaddress", ipaddress, (std::string)DEFAULT_SICK_IP_ADDRESS);
  nh_ns.param("inverted", inverted, false);
  nh_ns.param("publish_odom",publish_odom,false);
  nh_ns.param("perform_mapping", do_mapping, true);
  nh_ns.param<std::string>("frame_id", frame_id, "map");
  nh_ns.param<std::string>("sick_frame_id", sick_frame_id, "sick_nav350");
  nh_ns.param<std::string>("scan_frame_id", scan_frame_id, sick_frame_id);
  nh_ns.param<std::string>("reflector_frame_id", reflector_frame_id, "nav350");
  nh_ns.param<std::string>("reflector_child_frame_id", reflector_child_frame_id, "reflector");
  nh_ns.param<std::string>("target_frame_id",target_frame_id,sick_frame_id);
  nh_ns.param<std::string>("mobile_base_frame_id",mobile_base_frame_id,mobile_base_frame_id);

  nh_ns.param("wait_command", wait, 1);
  nh_ns.param("mask_command", mask, 2);

  nh_ns.param("timer_smoothing_factor", smoothing_factor, 0.97);
  nh_ns.param("timer_error_threshold", error_threshold, 0.5);
  nh_ns.param("resolution", sick_step_angle, 1.0);
  nh_ns.param("start_angle", active_sector_start_angle, 0.);
  nh_ns.param("stop_angle", active_sector_stop_angle, 360.);
  nh_ns.param("scan_rate", sick_motor_speed, 5);
  /* Define buffers for return values */
  double range_values[SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};
  int intensity_values[SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};
  /* Define buffers to hold sector specific data */
  unsigned int num_measurements = {0};
  unsigned int sector_start_timestamp = {0};
  unsigned int sector_stop_timestamp = {0};
  double sector_step_angle = {0};
  double sector_start_angle = {0};
  double sector_stop_angle = {0};

  std::vector<tf::TransformBroadcaster> landmark_broadcasters;
  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformListener tf_listerner;
  /* Instantiate the object */
  SickNav350 sick_nav350(ipaddress.c_str(),port);
  double last_time_stamp=0;
  try
  {
    /* Initialize the device */
    sick_nav350.Initialize();
    sick_nav350.GetSickIdentity();
    // TODO: do some calls to setup the device - e.g. scan rate. Configure mapping. Configure reflectors/landmarks

    if (do_mapping)
    {
      sick_nav350.SetOperatingMode((int)OperatingModes::MAPPING);
      sick_nav350.DoMapping();
      sick_nav350.SetOperatingMode((int)OperatingModes::STANDBY);
      ROS_INFO_STREAM("Sicknav50 Mapping Completed");
    }
    try
    {
      sick_nav350.SetOperatingMode((int)OperatingModes::STANDBY);
      sick_nav350.SetScanDataFormat();
      sick_nav350.SetOperatingMode(op_mode);
    }
    catch (...)
    {
      ROS_ERROR("Configuration error");
      return -1;
    }

    smoothtime smoothtimer;
    averager avg_fulldur, avg_scandur;
    smoothtimer.set_smoothing_factor(smoothing_factor);
    smoothtimer.set_error_threshold(error_threshold);
    ros::Time last_start_scan_time;
    unsigned int last_sector_stop_timestamp = 0;
    double full_duration;
    ros::Rate loop_rate(8);
    std::vector<tf::TransformBroadcaster> odom_broadcasters;

    // looking up transform from sicknav350 to target frame
    if(target_frame_id == sick_frame_id)
    {
      sickn350_to_target_tf.setIdentity();
    }
    else
    {
      try
      {
        std::string error_msg;
        ros::Time current_time = ros::Time(0);
        if(!tf_listerner.waitForTransform(
            target_frame_id,sick_frame_id,ros::Time(0),ros::Duration(TRANSFORM_TIMEOUT),
            ros::Duration(POLLING_DURATION),&error_msg))
        {
          ROS_ERROR_STREAM("Transform lookup timed out, error msg: "<<error_msg);
          return -1;
        }

        tf_listerner.lookupTransform(sick_frame_id,target_frame_id,current_time,sickn350_to_target_tf);
      }
      catch(tf::LookupException &exp)
      {
        ROS_ERROR_STREAM("Transform lookup between "<<sick_frame_id<<" and "<<target_frame_id<<" failed, exiting");
        return -1;
      }
    }


    ros::Time previous_time = ros::Time::now()-ros::Duration(0.5f);
    tf::Transform mobile_base_current_tf = tf::Transform::getIdentity();
    tf::Transform mobile_base_prev_tf = tf::Transform::getIdentity();
    nav_msgs::Odometry odom_msg;

    if(publish_odom)
    {
      odom_pub = nh.advertise<nav_msgs::Odometry>(ODOM_TOPIC, 1);

      if(mobile_base_frame_id == "")
      {
        ROS_ERROR_STREAM("Frame id for mobile base was not set in the parameter list");
        return -1;
      }
      odom_msg.header.frame_id = frame_id;
      odom_msg.child_frame_id = mobile_base_frame_id;

      try
      {
        std::string error_msg;
        ros::Time current_time = ros::Time(0);
        if(!tf_listerner.waitForTransform(
            target_frame_id,mobile_base_frame_id,ros::Time(0),ros::Duration(TRANSFORM_TIMEOUT),
            ros::Duration(POLLING_DURATION),&error_msg))
        {
          ROS_ERROR_STREAM("Transform lookup timed out, error msg: "<<error_msg);
          return -1;
        }

        tf_listerner.lookupTransform(target_frame_id,mobile_base_frame_id,current_time,target_to_mobile_base_tf);
      }
      catch(tf::LookupException &exp)
      {
        ROS_ERROR_STREAM("Transform lookup between "<<mobile_base_frame_id<<" and "<<target_frame_id<<" failed, exiting");
        return -1;
      }

    }

    while (ros::ok())
    {
      //Grab the measurements (from all sectors)
      if (op_mode==3)
      {
      ROS_INFO_STREAM("Getting landmark data");
      sick_nav350.GetDataLandMark(1,1);
      }
      else if (op_mode==4)
      {
      ROS_DEBUG_STREAM("Getting nav data");
      sick_nav350.GetDataNavigation(wait,mask);
      }
      else
      {
        ROS_INFO_STREAM("Selected operating mode does not return data... try again");
        return -1;
      }
      ROS_DEBUG_STREAM("Getting sick range/scan measurements");
      sick_nav350.GetSickMeasurements(range_values,
                                      intensity_values,
                                      &num_measurements,
                                      &sector_step_angle,
                                      &sector_start_angle,
                                      &sector_stop_angle,
                                      &sector_start_timestamp,
                                      &sector_stop_timestamp);


      /*
       * Get nav localization data and publish /map to /odom transform
       */
      double x1=(double) sick_nav350.PoseData_.x;
      double y1=(double) sick_nav350.PoseData_.y;
      double phi1=sick_nav350.PoseData_.phi;
      ROS_DEBUG_STREAM("NAV350 pose in x y alpha:"<<x1<<" "<<y1<<" "<<phi1/1000.0);
      tf::Transform odom_to_sick_tf;
      tf::Transform odom_to_target_tf;
      tf::Quaternion odomquat=tf::createQuaternionFromYaw(DEG2RAD(phi1/1000.0));
      odomquat.inverse();
      odom_to_sick_tf.setRotation(odomquat);
      odom_to_sick_tf.setOrigin(tf::Vector3(-x1 / 1000, -y1/ 1000, 0.0));

      // converting to target frame
      odom_to_target_tf = odom_to_sick_tf * sickn350_to_target_tf;

      ROS_DEBUG_STREAM("Sending transform from "<<frame_id<<" to "<<target_frame_id);
      odom_broadcaster.sendTransform(tf::StampedTransform(odom_to_target_tf, ros::Time::now(),
                                                                        frame_id, target_frame_id));

      // publishing odometry
      if(publish_odom)
      {
        mobile_base_current_tf = odom_to_target_tf*target_to_mobile_base_tf;
        createOdometryMessage(ros::Time::now() - previous_time,mobile_base_prev_tf,mobile_base_current_tf,odom_msg);
        mobile_base_prev_tf = mobile_base_current_tf;
        previous_time = ros::Time::now();
        odom_pub.publish(odom_msg);
      }

      /*
       * Get landmark data and broadcast transforms
       */
      int num_reflectors=sick_nav350.PoseData_.numUsedReflectors;
      int number_reflectors=sick_nav350.ReflectorData_.num_reflector;
      std::vector<double> Rx(number_reflectors), Ry(number_reflectors);
      ROS_DEBUG_STREAM("NAV350 # reflectors seen:"<<number_reflectors);
      ROS_DEBUG_STREAM("NAV350 # reflectors used:"<<num_reflectors);
      for (int r=0;r<number_reflectors; r++)
      {
        Rx[r]=(double) sick_nav350.ReflectorData_.x[r];
        Ry[r]=(double) sick_nav350.ReflectorData_.y[r];
        ROS_DEBUG_STREAM("Reflector "<<r<<" x pos: "<<Rx[r]<<" y pos: "<<Ry[r]);
      }
      landmark_broadcasters.resize(number_reflectors);
      PublishReflectorTransform(Rx,Ry,DEG2RAD(phi1/1000.0),landmark_broadcasters,reflector_frame_id,reflector_child_frame_id);

      /*
       * Get Scan data and publish scan
       */
      if (sector_start_timestamp<last_time_stamp)
      {
        loop_rate.sleep();
        ros::spinOnce();
        continue;
      }
      last_time_stamp=sector_start_timestamp;
      ros::Time end_scan_time = ros::Time::now();

      double scan_duration = (sector_stop_timestamp - sector_start_timestamp) * 1e-3;
      avg_scandur.add_new(scan_duration);
      scan_duration = 0.125;//avg_scandur.get_mean();
      if (last_sector_stop_timestamp == 0) {
        full_duration = 1./((double)sick_motor_speed);
      } else {
        full_duration = (sector_stop_timestamp - last_sector_stop_timestamp) * 1e-3;
      }
      avg_fulldur.add_new(full_duration);
      full_duration = avg_fulldur.get_mean();

      ros::Time smoothed_end_scan_time = smoothtimer.smooth_timestamp(end_scan_time, ros::Duration(full_duration));
      ros::Time start_scan_time = smoothed_end_scan_time - ros::Duration(scan_duration);
      sector_start_angle-=180;
      sector_stop_angle-=180;
      publish_scan(&scan_pub, range_values, num_measurements, intensity_values,
                   num_measurements, start_scan_time, scan_duration, inverted,
                   DEG2RAD((float)sector_start_angle), DEG2RAD((float)sector_stop_angle), scan_frame_id, sector_start_timestamp,&scan_pub1);



       /*ROS_INFO_STREAM/*DEBUG_STREAM*//*("Num meas: " << num_measurements
       << " smoothed start T: " << start_scan_time
       << " smoothed rate: " << 1./(start_scan_time - last_start_scan_time).toSec()
       << " raw start T: " << sector_start_timestamp
       << " raw stop T: " << sector_stop_timestamp
       << " dur: " << full_duration
       << " step A: " << sector_step_angle
       << " start A: " << sector_start_angle
       << " stop A: " << sector_stop_angle);
       //last_start_scan_time = start_scan_time;
       //last_sector_stop_timestamp = sector_stop_timestamp;*/
      loop_rate.sleep();
      ros::spinOnce();

    }

    /* Uninitialize the device */
    try
    {
      sick_nav350.Uninitialize();
    }
    catch(...)
    {
      cerr << "Uninitialize failed!" << endl;
      return -1;
    }
  }
  catch(...)
  {
    ROS_ERROR("Error");
    return -1;
  }
  return 0;
}
