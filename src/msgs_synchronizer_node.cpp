#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <upo_msgs/PersonPoseArrayUPO.h>
#include <spencer_tracking_msgs/TrackedPersons.h>
#include <iostream>
#include <math.h>
//Visualization
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, spencer_tracking_msgs::TrackedPersons, sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> msgSyncPolicy;

class msgs_synchronizer
{
private:
  message_filters::Subscriber<nav_msgs::Odometry>* odom_sub_;
  message_filters::Subscriber<spencer_tracking_msgs::TrackedPersons>* people_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2>* obstale_sub_;
  message_filters::Subscriber<geometry_msgs::PoseStamped>* odom_map_sub_;
  message_filters::Synchronizer<msgSyncPolicy>* sync_;
  ros::NodeHandle n_;
  ros::Publisher pub_posestamped_;
  ros::Publisher pub_twist_;
  ros::Publisher pub_;
  ros::Publisher people_arrow_pub_;
  ros::Publisher people_cyl_pub_;
  ros::Publisher pub_obstacles_;
  ros::Publisher pub_robot_map_;
  ros::Publisher pub_goal_;
  
  
public:
  msgs_synchronizer()
  {}
  ~msgs_synchronizer()
  {}
  
  void init()
  {
    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(n_, "/odom", 1);
    people_sub_ = new message_filters::Subscriber<spencer_tracking_msgs::TrackedPersons>(n_, "/spencer/perception/tracked_persons_confirmed_by_HOG_or_upper_body", 1);
    obstale_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(n_,"/obstacles",1);
    odom_map_sub_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(n_,"/robot_map",1);
    sync_ = new message_filters::Synchronizer<msgSyncPolicy>(msgSyncPolicy(10), *odom_sub_, *people_sub_, *obstale_sub_, *odom_map_sub_);
    sync_->registerCallback(boost::bind(&msgs_synchronizer::combineCallback, this, _1, _2, _3, _4));
    pub_posestamped_ = n_.advertise<geometry_msgs::PoseStamped>("/robot",1000);
    pub_twist_ = n_.advertise<geometry_msgs::Twist>("/vel",1000);
    pub_ = n_.advertise<upo_msgs::PersonPoseArrayUPO>("/people",1000);
    people_arrow_pub_ = n_.advertise<visualization_msgs::MarkerArray>("/people/people_arrow_markers", 1);
    people_cyl_pub_ = n_.advertise<visualization_msgs::MarkerArray>("/people/people_cylinders_markers", 1);
    pub_obstacles_ = n_.advertise<sensor_msgs::PointCloud2>("/obstacles_new", 1);
    pub_robot_map_ = n_.advertise<geometry_msgs::PoseStamped>("/robot_map_new", 1);
    pub_goal_ = n_.advertise<geometry_msgs::PoseStamped>("/goal", 1);
  }
  
  void combineCallback(const nav_msgs::Odometry::ConstPtr& odom_msg, const spencer_tracking_msgs::TrackedPersons::ConstPtr& people_msgs, const sensor_msgs::PointCloud2::ConstPtr& obstacles_msgs, const geometry_msgs::PoseStamped::ConstPtr& odom_map_msgs)
  {
//     std::cout<<"called !!!!!!!!!!!!!!!"<<std::endl;
//     ros::Time now = odom_msg->header.stamp;
//     std::cout<<now.sec<<": "<<now.nsec<<std::endl;
//     now = people_msgs->header.stamp;
//     std::cout<<now.sec<<": "<<now.nsec<<std::endl;
//     now = obstacles_msgs->header.stamp;
//     std::cout<<now.sec<<": "<<now.nsec<<std::endl;
//     now = odom_map_msgs->header.stamp;
//     std::cout<<now.sec<<": "<<now.nsec<<std::endl;
    
    //odom_msg transform
    geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::Twist vel;
    pose_stamped.header = odom_msg->header;
    pose_stamped.pose = odom_msg->pose.pose;
    vel = odom_msg->twist.twist;
    
    //people_msgs transform
    upo_msgs::PersonPoseArrayUPO people_upo;
    people_upo.header = people_msgs->header;
    int size = people_msgs->tracks.size();
    people_upo.size = size;
    for(int i=0;i<size;i++)
    {
      spencer_tracking_msgs::TrackedPerson people_spencer = people_msgs->tracks[i];
      upo_msgs::PersonPoseUPO p_upo;
      p_upo.header = people_msgs->header;
      p_upo.id = people_spencer.track_id;
      p_upo.position = people_spencer.pose.pose.position;
      p_upo.orientation = people_spencer.pose.pose.orientation;
      p_upo.vel = sqrt((people_spencer.twist.twist.linear.x * people_spencer.twist.twist.linear.x) + (people_spencer.twist.twist.linear.y * people_spencer.twist.twist.linear.y));
      people_upo.personPoses.push_back(p_upo);
    }
    
//     pub_goal_.publish(goal);     //添加发布goal消息的部分
    pub_robot_map_.publish(*odom_map_msgs);
    pub_obstacles_.publish(*obstacles_msgs);
    pub_.publish(people_upo);
    visualize_people(people_upo);
    pub_posestamped_.publish(pose_stamped);
    pub_twist_.publish(vel);
  }
  
  
  void visualize_people(upo_msgs::PersonPoseArrayUPO peop)
  {
    visualization_msgs::MarkerArray arrowArray;	
	visualization_msgs::MarkerArray cylinderArray;
	ros::Time time = ros::Time::now();
	for(unsigned int i=0; i<peop.size; i++)
	{
		visualization_msgs::Marker markerDel;
		//Delete previous markers
		markerDel.action = 3; //visualization_msgs::Marker::DELETEALL;
		arrowArray.markers.push_back(markerDel);
		cylinderArray.markers.push_back(markerDel);
		people_arrow_pub_.publish(arrowArray);
		people_cyl_pub_.publish(cylinderArray);

		// Fill up arrow marker information
		visualization_msgs::Marker marker;
		marker.header.frame_id = peop.header.frame_id;
		marker.header.stamp = time; 
		marker.ns = "people_pose";
		marker.id = (i+1);
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position = peop.personPoses[i].position;
		marker.pose.orientation = peop.personPoses[i].orientation;
		marker.pose.position.z = 1.0; 
		marker.scale.x = 0.6; 
		marker.scale.y = 0.1; 
		marker.scale.z = 0.4; 
		marker.color.a = 0.8;
		marker.color.r = 0.0; 
		marker.color.g = 1.0; 
		marker.color.b = 0.0; 
		marker.lifetime = ros::Duration();
		arrowArray.markers.push_back(marker);

		// Fill up cylinder marker information
		marker.header.frame_id = peop.header.frame_id;
		marker.header.stamp = time; 
		marker.ns = "people_pose";
		marker.id = (i+1);
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position = peop.personPoses[i].position;
		marker.pose.orientation = peop.personPoses[i].orientation;
		marker.pose.position.z = 0.50;
		marker.scale.x = 0.5; 
		marker.scale.y = 0.5; 
		marker.scale.z = 1.5; 
		marker.color.a = 0.8;
		marker.color.r = 0.0; 
		marker.color.g = 1.0; 
		marker.color.b = 0.0; 
		marker.lifetime = ros::Duration();
		cylinderArray.markers.push_back(marker);

	}
	people_arrow_pub_.publish(arrowArray);
	people_cyl_pub_.publish(cylinderArray);
  }
};




int main(int argc, char ** argv)
{
  ros::init(argc, argv, "msgs_synchronizer");
  msgs_synchronizer msgs_syn;
  msgs_syn.init();
  
  ros::spin();
  return 0;
}