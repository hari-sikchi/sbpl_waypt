#include <waypointnav.hpp>

using namespace std;

//max define
ofstream path;


 void wayPointNav::botpos_sub(const nav_msgs::Odometry botpos){
  bot_pos = msgtarget;
   
  tf::Quaternion q(botpos.pose.pose.orientation.x, botpos.pose.pose.orientation.y, botpos.pose.pose.orientation.z, botpos.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    bot_yaw=yaw;
}


void wayPointNav::prop_target(const geometry_msgs::PoseStamped msgtarget){
  target_pos = msgtarget;
  
  tf::Quaternion q(msgtarget.pose.orientation.x, msgtarget.pose.orientation.y, msgtarget.pose.orientation.z, msgtarget.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw,target_yaw;
  m.getRPY(roll, pitch, yaw);
  target_yaw=yaw;
}

void laserscan_sub(const sensor_msgs::LaserScan msg){
  scandata = msg;
}

void wayPointNav::update_map(){

      //path.open("my_env.cfg",ios_base::app);
      /*convert laserscan data to point cloud.ref:http://wiki.ros.org/laser_geometry*/
        int i=0;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener listener_;

        if(!listener_.waitForTransform(scandata.header.frame_id,"/base_link",scandata.header.stamp + 
          ros::Duration().fromSec(scandata.ranges.size()*scandata.time_increment),ros::Duration(1.0)))
          {return;}
              sensor_msgs::PointCloud cloud;
              projector_.transformLaserScanToPointCloud("/base_link",scandata,cloud,listener_);

              /*transform the point cloud into odom frame*/
              long long int Numcldpts=cloud.points.size();
              for(int i =0;i<Numcldpts;i++){
                 geometry_msgs::PointStamped pt;
                 geometry_msgs::PointStamped pt_transformed;
                 pt.header = cloud.header;
                 pt.point.x = cloud.points[i].x;
                 pt.point.y = cloud.points[i].y;
              
                 try{
                    listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(10.0) );
                    listener.transformPose("/odom", pt, pt_transformed);
                  } catch (tf::TransformException ex) {
                      ROS_ERROR("%s",ex.what());
                    }
                 //listener.transformPoint("/odom", pt, pt_transformed);
                 cout<<pt_transformed.x<<" "<<pt_transformed.y<<endl;
                 glob_map[pt_transformed.x][pt_transformed.y]=1;
                 ///doubt for infff
              }
}

sbplWaypointNav::sbplWaypointNav(ros::NodeHandle &node_handle){
    ros::NodeHandle node_handle;
    odom_sub = node_handle.subscribe("/odometry/filtered", buffer_size, &sbplWaypointNav::botpos_sub, this);
    target_sub = node_handle.subscribe("/waypoint_navigator/proposed_target", buffer_size, &sbplWaypointNav::proptarget_sub, this);
    scan_sub = node_handle.subscribe("/scan", buffer_size, &sbplWaypointNav::laserscan_sub, this);
    //lane_sub = node_handle.subscribe("/cloud_data", buffer_size, &sbplWaypointNav::pointcloud, this);
}