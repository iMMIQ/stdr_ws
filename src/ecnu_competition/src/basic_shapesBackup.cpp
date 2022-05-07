#include <cservice/c_srv.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class odomObserve {
 public:
  odomObserve() {
    // Topic you want to publish
    // pub_ = n_.advertise<PUBLISHED_MESSAGE_TYPE>("/published_topic", 1);
    shape = visualization_msgs::Marker::CUBE;
    for (int i = 0; i < 5; i++) {
      flags[i] = false;
      rect[i][0] = i + 2;
      rect[i][1] = i + 3;
    }

    client = n.serviceClient<cservice::c_srv>("recive_a_bool");
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("MarkerArray", 5);
    // Topic you want to subscribe
    // sub_ = n_.subscribe("/subscribed_topic", 1,
    // &SubscribeAndPublish::callback, this);
    odom_sub =
        n.subscribe("robot0/odom", 100, &odomObserve::callback_odom, this);
  }

  void callback_odom(
      const nav_msgs::Odometry::ConstPtr& odom)  //订阅/odom主题回调函数
  {
    float x, y;
    x = odom->pose.pose.position.x;
    y = odom->pose.pose.position.y;

    fillMarkers(x, y);
    // Publish the marker
    if (marker_pub.getNumSubscribers() < 1) {
      if (!ros::ok()) {
        return;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    marker_pub.publish(markerList);
  }

  bool checkIfGet(int i) {
    cservice::c_srv srv;
    srv.request.a = flags[i];
    client.call(srv);
    return flags[i];
  }
  void rectChange(float x, float y, int i) {
    if (x < rect[i][0] + 0.5 && x > rect[i][0] - 0.5 & y < rect[i][1] + 0.5 &&
        y > rect[i][1] - 0.5)
      flags[i] = true;
  }
  void fillMarkers(float x, float y) {
    markerList.markers.clear();
    for (int i = 0; i < 5; i++) {
      visualization_msgs::Marker marker;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.5f;
      // Set the frame ID and timestamp.  See the TFbase_linkutorials for
      // information on these.
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      // Set the namespace and id for this marker.  This serves to create a
      // unique ID Any marker sent with the same namespace and id will overwrite
      // the old one
      marker.ns = "basic_shapes";
      marker.id = i;

      // Set the marker type.  Initially this is CUBE, and cycles between that
      // and SPHERE, ARROW, and CYLINDER
      marker.type = shape;

      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo:
      // 3 (DELETEALL)
      marker.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the
      // frame/time specified in the header
      marker.pose.position.x = rect[i][0];
      marker.pose.position.y = rect[i][1];
      marker.pose.position.z = 0;

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;
      rectChange(x, y, i);
      // Set the color -- be sure to set alpha to something non-zero!
      if (checkIfGet(i)) {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5f;
      }

      marker.lifetime = ros::Duration();
      markerList.markers.push_back(marker);
    }
  }

 private:
  ros::NodeHandle n;
  ros::Publisher marker_pub;
  ros::Subscriber odom_sub;
  ros::ServiceClient client;
  visualization_msgs::MarkerArray markerList;
  // visualization_msgs::Marker marker;
  uint32_t shape = visualization_msgs::Marker::CUBE;
  bool flags[5];
  float rect[5][2];  // x,y
};                   // End of class SubscribeAndPublish

int main(int argc, char** argv) {
  // Initiate ROS
  ros::init(argc, argv, "basic_shapes");

  // Create an object of class SubscribeAndPublish that will take care of
  // everything
  odomObserve oo;

  ros::spin();

  return 0;
}
