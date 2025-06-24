#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>
#include <builtin_interfaces/msg/time.hpp>

//using namespace std::placeholders;

class GroundTruthBridge : public rclcpp::Node
{
public:
  GroundTruthBridge(const std::string &robot_name, const rclcpp::NodeOptions & options)
  : Node("ground_truth_bridge", options), robot_name_(robot_name)
  {
    // Publisher
    pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("gt_pose", 10);

    //Subscriber
    success = gz_node_.Subscribe("/world/default/pose/info", &GroundTruthBridge::gzCallback, this);

    if (!success)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to /world/default/pose/info");
    }

  }

private:

  void gzCallback(const gz::msgs::Pose_V &msg)
  {
    builtin_interfaces::msg::Time ros_time;
    const auto &gz_stamp = msg.header().stamp();
    ros_time.sec = static_cast<int32_t>(gz_stamp.sec());
    ros_time.nanosec = static_cast<uint32_t>(gz_stamp.nsec());

    for (int i = 0; i < msg.pose_size(); ++i)
    {
      const auto &pose = msg.pose(i);
      if (pose.name() == robot_name_)
      {
        geometry_msgs::msg::PoseStamped ros_pose;
        ros_pose.header.stamp = ros_time;
        ros_pose.header.frame_id = "world";
        ros_pose.pose.position.x = pose.position().x();
        ros_pose.pose.position.y = pose.position().y();
        ros_pose.pose.position.z = pose.position().z();
        ros_pose.pose.orientation.x = pose.orientation().x();
        ros_pose.pose.orientation.y = pose.orientation().y();
        ros_pose.pose.orientation.z = pose.orientation().z();
        ros_pose.pose.orientation.w = pose.orientation().w();
        pub_->publish(ros_pose);
      }
    }
  }

  bool success;
  std::string robot_name_;
  gz::transport::Node gz_node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  if (argc < 2)
  {
    std::cerr << "Usage: ros2 run <pkg> gt_bridge_node <robot_name>" << std::endl;
    return 1;
  }
  std::string robot_name = argv[1];
  // Create node options and set 'use_sim_time' parameter to true
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  auto node = std::make_shared<GroundTruthBridge>(robot_name, options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
