// standard headers
#include <stdio.h>
#include <vector>
#include <string>

// ROS headers
#include <ros/ros.h>
#include <ros/message_operations.h>
#include <visualization_msgs/MarkerArray.h>

// local headers
#include "phase_space/PhaseSpaceMarkerArray.h"

namespace phase_space {

class PhaseSpaceViz
{
  public:

    // the node handle
    ros::NodeHandle nh_;

    // node handle in the private namespace
    ros::NodeHandle priv_nh_;
    
    // publishers
    ros::Publisher pub_rviz_markers_;

    // subscribers
    ros::Subscriber sub_phase_space_markers_;

    // the rviz markers
    visualization_msgs::MarkerArray rviz_markers_;
    visualization_msgs::Marker rviz_marker_;
    visualization_msgs::Marker id_marker_;
    
  public:

    // callback functions
    void publishRVIZMarkers(const phase_space::PhaseSpaceMarkerArray& array);

    // constructor
    PhaseSpaceViz(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {

        // initialize the RVIZ markers
        // the leds
        rviz_marker_.ns = "/phase_space/markers";
        rviz_marker_.type = visualization_msgs::Marker::SPHERE;
        rviz_marker_.action = visualization_msgs::Marker::ADD;
        rviz_marker_.scale.x = 0.01;
        rviz_marker_.scale.y = 0.01;
        rviz_marker_.scale.z = 0.01;
        rviz_marker_.pose.orientation.x = 0.0;
        rviz_marker_.pose.orientation.y = 0.0;
        rviz_marker_.pose.orientation.z = 0.0;
        rviz_marker_.pose.orientation.w = 1.0;
        rviz_marker_.color.r = 1.0f;
        rviz_marker_.color.g = 0.0f;
        rviz_marker_.color.b = 0.0f;
        rviz_marker_.color.a = 1.0;
        rviz_marker_.lifetime = ros::Duration();
        
        // and its id
        id_marker_.ns = "/phase_space/id";
        id_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        id_marker_.action = visualization_msgs::Marker::ADD;
        id_marker_.color.r = 1.0f;
        id_marker_.color.g = 1.0f;
        id_marker_.color.b = 1.0f;
        id_marker_.color.a = 1.0;
        id_marker_.scale.z = 0.03;
        
        // advertise topics
        pub_rviz_markers_ = nh.advertise<visualization_msgs::MarkerArray>(nh.resolveName("/rviz_markers"), 10);

        // resolve subscriber topics
        sub_phase_space_markers_ = nh_.subscribe(nh_.resolveName("/phase_space_markers"), 10, &PhaseSpaceViz::publishRVIZMarkers, this);
    }
    
    //! Empty stub
    ~PhaseSpaceViz() {}

};

// this function is called as fast as ROS can from the main loop directly
void PhaseSpaceViz::publishRVIZMarkers(const phase_space::PhaseSpaceMarkerArray& array)
{
    for(int i = 0; i< array.markers.size() ; ++i)
    {
        // fill the led marker
        rviz_marker_.header = array.header;
        rviz_marker_.id = array.markers[i].id;
        rviz_marker_.pose.position = array.markers[i].point;

        // fill the led id
        id_marker_.header = array.header;
        id_marker_.text = std::to_string(array.markers[i].id);
        id_marker_.id = array.markers[i].id;
        id_marker_.pose.position = array.markers[i].point;

        rviz_markers_.markers.push_back(rviz_marker_);
        rviz_markers_.markers.push_back(id_marker_);
    }
    pub_rviz_markers_.publish(rviz_markers_);
    rviz_markers_.markers.clear();
}

} // namespace phase_space

int main(int argc, char **argv)
{
    ros::init(argc, argv, "phase_space_viz_publisher");
    ros::NodeHandle nh;

    phase_space::PhaseSpaceViz node(nh);

    ros::spin();

    return 0;
}
