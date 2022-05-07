#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
//#include <cservice/c_srv.h>
#include<iostream>
class odomObserve
{
public:
  odomObserve()
  {
    //Topic you want to publish
    //pub_ = n_.advertise<PUBLISHED_MESSAGE_TYPE>("/published_topic", 1);
    shape = visualization_msgs::Marker::CUBE;
    //shape = visualization_msgs::Marker::TEXT_VIEW_FACING;
    for(int i=0;i<12;i++){
      flags[i] = false;
      //rect[i][0] = (i+2)/2;
      //rect[i][1] =  (i+3)/2;
    }
    
    rect[0][0] = 4.5;
    rect[0][1] =  10;
    rect[1][0] = 1.5;
    rect[1][1] =  9;
    rect[2][0] = 3;
    rect[2][1] =  7.5;
    
    rect[3][0] = 2.5;
    rect[3][1] =  5.3;
    rect[4][0] = 2.5;
    rect[4][1] =  2.5;
    rect[5][0] = 6;
    rect[5][1] =  6.5;
    
    rect[6][0] =  6;
    rect[6][1] = 2.5;
    rect[7][0] = 9.0;
    rect[7][1] = 0.6;
    rect[8][0] = 9.0;
    rect[8][1] = 6.2;
    
    rect[9][0] = 10;
    rect[9][1] = 7.5;
    rect[10][0] = 10.0;
    rect[10][1] = 10.0;
    rect[11][0] = 7.0;
    rect[11][1] = 10.0;
    
    start_flag = false;
    end_flag = false;
    
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("MarkerArray", 20);
    
    num_pub = n.advertise<visualization_msgs::MarkerArray>("NumberArray", 20);
    
    //Topic you want to subscribe
    //sub_ = n_.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback, this);
    odom_sub = n.subscribe("robot0/odom", 100, &odomObserve::callback_odom,this);
  }
 
  void callback_odom(const nav_msgs::Odometry::ConstPtr& odom)//订阅/odom主题回调函数
  {    
  
    float x,y; 
    x = odom -> pose.pose.position.x;
    y = odom -> pose.pose.position.y;
   
    
    
    fillMarkers(x,y);
    fillMarkers2(x,y);
    // Publish the marker
    if (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return ;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    
    marker_pub.publish(markerList);
    
    num_pub.publish(numberList);

    
  }
 

  bool checkIfGet(int i){
  	return flags[i];
  }
  void rectChange(float x, float y,int i){
    if(x< rect[i][0]+0.25 &&x>rect[i][0]-0.25 && y<rect[i][1]+0.25 && y>rect[i][1]-0.25){
      if(start_flag == false && i == 0){
        start_flag = true; 
        start_time  =  ros::Time::now().toSec();
        ROS_WARN_ONCE("game start time now");
      }
      else if(end_flag == false && i == 11){
        end_flag = true;
        use_time = ros::Time::now().toSec() - start_time;
        ROS_WARN_ONCE("game end time now, you use time [%lf]",use_time);
        std::cout<<use_time; 
      }
      flags[i] = true;  
         
    }
      	  	
  }
  
  void fillMarkers(float x, float y){
    markerList.markers.clear();
    
    for(int i=0;i<12;i++){
      visualization_msgs::Marker marker;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.2f;
      // Set the frame ID and timestamp.  See the TFbase_linkutorials for information on these.
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "basic_shapes";
      marker.id = i;

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type = shape;

      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      marker.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = rect[i][0];
      marker.pose.position.y = rect[i][1];
      marker.pose.position.z = 0;
    
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.5;
      marker.scale.y = 0.5;
      marker.scale.z = 0.5;
      rectChange(x,y,i);
      // Set the color -- be sure to set alpha to something non-zero!
      if(checkIfGet(i)){
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.2f;
      }
      
      //marker.text = "tzz";
      marker.lifetime = ros::Duration();
      
      markerList.markers.push_back(marker);
      
     
      
      
    }
  } 
  
  void fillMarkers2(float x, float y){
    
    numberList.markers.clear();
    for(int i=0;i<12;i++){
      visualization_msgs::Marker marker;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;
      // Set the frame ID and timestamp.  See the TFbase_linkutorials for information on these.
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "basic_shapes";
      
      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type = shape;

      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      marker.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = rect[i][0];
      marker.pose.position.y = rect[i][1];
      marker.pose.position.z = 0;
    
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.5;
      marker.scale.y = 0.5;
      marker.scale.z = 0.5;
      rectChange(x,y,i);
      // Set the color -- be sure to set alpha to something non-zero!
      if(checkIfGet(i)){
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
      }
      
      //marker.text = "tzz";
      marker.lifetime = ros::Duration();
      std::ostringstream str;

      str<<i;

      marker.text=str.str();
     
        marker.type  = visualization_msgs::Marker::TEXT_VIEW_FACING;  
        marker.color.a = 1.0f;
        marker.id = i+12;
        numberList.markers.push_back(marker);
     
      
      
    }
  } 
    
private:
  ros::NodeHandle n; 
  ros::Publisher marker_pub,num_pub;
  ros::Subscriber odom_sub;
  //ros::ServiceClient client; 
  visualization_msgs::MarkerArray markerList, numberList;
  //visualization_msgs::Marker marker;
  uint32_t shape = visualization_msgs::Marker::CUBE;
  bool flags[12],start_flag,end_flag;
  float rect[12][2];//x,y
  double use_time,start_time;
};//End of class SubscribeAndPublish



int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "basic_shapes");
 
  //Create an object of class SubscribeAndPublish that will take care of everything
  odomObserve oo;

  ros::spin();
 
  return 0;

}




