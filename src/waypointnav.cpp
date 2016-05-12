#include <waypointnav.hpp>

using namespace std;

//max define
ofstream path;
//ifstream readpath;


 void sbplWaypointNav::botpos_sub(const nav_msgs::Odometry botpos){
  bot_pos = botpos;
   
  tf::Quaternion q(botpos.pose.pose.orientation.x, botpos.pose.pose.orientation.y, botpos.pose.pose.orientation.z, botpos.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    bot_yaw=yaw;
}


void sbplWaypointNav::proptarget_sub(const geometry_msgs::PoseStamped msgtarget){
  target_pos = msgtarget;
  
  tf::Quaternion q(msgtarget.pose.orientation.x, msgtarget.pose.orientation.y, msgtarget.pose.orientation.z, msgtarget.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw,target_yaw;
  m.getRPY(roll, pitch, yaw);
  target_yaw=yaw;
}

void sbplWaypointNav::laserscan_sub(const sensor_msgs::LaserScan msg){
  scandata = msg;
}

void sbplWaypointNav::printdata(){
  //Mat img(mapsize,mapsize,CV_8UC3,Scalar(0,0,0));
  path.open("my_env1.cfg");
  string s[10];
  s[0]="discretization(cells): 2240 2240";
  s[1]="obsthresh: 1";
  s[2]="cost_inscribed_thresh: 1";
  s[3]="cost_possibly_circumscribed_thresh: 0";
  s[4]="cellsize(meters): 0.025";
  s[5]="nominalvel(mpersecs): 1.0";
  s[6]="timetoturn45degsinplace(secs): 2.0";
  s[7]="start(meters,rads): ";
  s[8]="end(meters,rads): ";
  s[9]="environment:";
  //cout<<"bot pose:"<<bot_pos.pose.pose.position.x <<" "<<bot_pos.pose.pose.position.y<<" "<<bot_yaw<<endl;
  //cout<<"transform: "<<target_base_link.pose.position.x<<" "<<target_base_link.pose.position.y<<" "<<base_link_yaw<<endl;
  //  cout<<"target pos:"<<target_base_link.pose.position.x+bot_pos.pose.pose.position.x<<" "<<target_base_link.pose.position.y+bot_pos.pose.pose.position.y<<" "<<base_link_yaw<<endl;

  path<<s[0]<<std::endl<<s[1]<<std::endl<<s[2]<<std::endl<<s[3]<<std::endl<<s[4]<<std::endl<<s[5]<<std::endl<<s[6]<<std::endl<<s[7]<<"56 28 0"<<std::endl<<s[8]<<-target_pos.pose.position.y+28<<" "<<target_pos.pose.position.x+28<<" "<<target_yaw<<std::endl<<s[9]<<std::endl;
  path.close();
}

void sbplWaypointNav::update_map(){

      //path.open("my_env.cfg",ios_base::app);
      /*convert laserscan data to point cloud.ref:http://wiki.ros.org/laser_geometry*/
        int i=0;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener listener_;
        /*if(!listener_.waitForTransform(scandata.header.frame_id,"/base_link",scandata.header.stamp + 
          ros::Duration().fromSec(scandata.ranges.size()*scandata.time_increment),ros::Duration(1.0)))
          {return;}*/
              sensor_msgs::PointCloud cloud;
              projector_.projectLaser(scandata, cloud);
              //projector_.transformLaserScanToPointCloud("/base_link",scandata,cloud,listener_);

              /*transform the point cloud into odom frame*/
              long long int Numcldpts=cloud.points.size();
              for(int i =0;i<Numcldpts;i++){
                 geometry_msgs::PoseStamped pt;
                 geometry_msgs::PoseStamped pt_transformed;
                 pt.header = cloud.header;
                 pt.pose.position.x = cloud.points[i].x;
                 pt.pose.position.y = cloud.points[i].y;
              
                 try{
                    listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(10.0) );
                    listener.transformPose("/odom", pt, pt_transformed);
                  } catch (tf::TransformException ex) {
                      ROS_ERROR("%s",ex.what());
                    }
                 //listener.transformPoint("/odom", pt, pt_transformed);
                    //print it
                 cout<<pt_transformed.pose.position.x<<" "<<pt_transformed.pose.position.y<<endl;
                 //update in glob map
                  int glob_x=pt_transformed.pose.position.x;
                  int glob_y=pt_transformed.pose.position.y;
                 glob_map[glob_x][glob_y]=1;
                 //write to config file
                 ///doubt for infff
              }
}

/*void sbplWaypointNav::map_zero(){
  static int i=0;
  i++;
  if(i>1){
    return;
  }
  path.open("my_env.cfg", ios_base::app);
  for(int m=0;m<mapsize;m++){
    for(int n=0;n<mapsize;n++){
      path<<"0 ";
    }
    path<<endl;
  }
}*/

void sbplWaypointNav::create_costmap(){
  path.open("my_env1.cfg",ios_base::app);
      int i,j;
      for(i=0;i<mapsize;i++){
         for(j=0;j<mapsize;j++){
            path<<glob_map[i][j]<<" ";
         }
         path<<std::endl;
      }
      path.close();
}

sbplWaypointNav::sbplWaypointNav(ros::NodeHandle &node_handle){
    odom_sub = node_handle.subscribe("/odometry/filtered", buffer_size, &sbplWaypointNav::botpos_sub, this);
    target_sub = node_handle.subscribe("/waypoint_navigator/proposed_target", buffer_size, &sbplWaypointNav::proptarget_sub, this);
    scan_sub = node_handle.subscribe("/scan", buffer_size, &sbplWaypointNav::laserscan_sub, this);
    //lane_sub = node_handle.subscribe("/cloud_data", buffer_size, &sbplWaypointNav::pointcloud, this);
}