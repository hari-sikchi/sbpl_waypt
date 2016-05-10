#include <waypointnav.hpp>

int main(int argc, char* argv[]) {

    std::string node_name = "sbpl_waypoint"; 
    ros::init(argc, argv, node_name);
    ros::NodeHandle node_handle;
    sbplLaneNav *sbpl_waypoint = new sbplWaypointNav(node_handle);
    ros::Rate loop_rate(10);
    while (ros::ok()){
    	
    	ros::spinOnce();
    	sbpl_waypoint->update_map();
    	loop_rate.sleep();
    }
    //ros::spin();
    return 0;
}
