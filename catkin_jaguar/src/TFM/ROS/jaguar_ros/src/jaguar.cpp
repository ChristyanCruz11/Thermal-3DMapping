#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/TwistStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float32.h>

namespace jaguar
{

class Jaguar
{
private:

  ros::NodeHandle node_handle_;
  ros::Subscriber dis_subscriber;
  ros::Publisher  dis_publisher;
  std::string     topicname_distance,
                  topicname_dis_pub,
                  framename_base_link,
                  framename_world;

public:
  void Usage(void)
  {
    printf("==================== Distance Usage ====================\n");
    printf("\n");    
  }
  
  Jaguar()
  {
    // Make a nodehandle for reading parameters from the local namespace.
    ros::NodeHandle _nh("~");
    // TODO Read Topicnames and framenames
    _nh.param<std::string>("topicNameDistance",        topicname_distance, "distance");
    _nh.param<std::string>("topicNameDistancePublisher",        topicname_dis_pub, "distance_publisher");
    _nh.param<std::string>("frameNameWorld",      framename_world,"world");
    _nh.param<std::string>("frameNameBaselink",   framename_base_link, 
                                                            "base_link");

    dis_subscriber = node_handle_.subscribe<sensor_msgs::Range>(topicname_distance, 1,
                       boost::bind(&Jaguar::disCallback, this, _1));
    dis_publisher = node_handle_.advertise<std_msgs::Float32>
                           (topicname_dis_pub, 10);
  }

  void disCallback(const sensor_msgs::RangeConstPtr &msg)
  {
    // Publish about distance
    float distance;
    distance = msg->range;
    std_msgs::Float32 pub_msg;
    pub_msg.data = distance;
    dis_publisher.publish(pub_msg);
  }

};

} // namespace teleop_crawler

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jaguar");

  jaguar::Jaguar jg;
  jg.Usage();
  ros::spin();

  return 0;
}