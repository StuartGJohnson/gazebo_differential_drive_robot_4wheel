// this does not work - known bug in gazebo garden
#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <ros_gz_interfaces/srv/delete_entity.hpp>
#include <ros_gz_interfaces/srv/spawn_entity.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/world_control.pb.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class ResetAndRespawnNode : public rclcpp::Node {
public:
  ResetAndRespawnNode()
  : Node("reset_and_respawn_node"), gz_node_(std::make_shared<gz::transport::Node>()) {
    std::string topic = "/world/default/control";
    pub = gz_node_->Advertise<gz::msgs::WorldControl>(topic);
    robot_name_ = "differential_drive_robot_4wheel";
    sdf_path_ = "/home/sjohnson/ros2_ws/src/gazebo_differential_drive_robot_4wheel/model/robot.urdf";  // Replace this
    initial_x_ = 0.0;
    initial_y_ = 0.0;

    timer_ = this->create_wall_timer(1s, std::bind(&ResetAndRespawnNode::runSequence, this));
  }

private:
  void runSequence() {
    timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Pausing simulation...");
    pauseSimulation();

    RCLCPP_INFO(this->get_logger(), "Deleting robot...");
    if (!deleteEntity(robot_name_)) return;

    RCLCPP_INFO(this->get_logger(), "Resetting simulation...");
    resetSimulation();

    RCLCPP_INFO(this->get_logger(), "Spawning robot...");
    if (!spawnEntity(robot_name_, sdf_path_, initial_x_, initial_y_)) return;

    RCLCPP_INFO(this->get_logger(), "Unpausing simulation...");
    unpauseSimulation();

    RCLCPP_INFO(this->get_logger(), "Reset and respawn complete. Exiting.");
    rclcpp::shutdown();
  }

  void pauseSimulation() {
    gz::msgs::WorldControl msg;
    msg.set_pause(true);
    pub.Publish(msg);
    std::this_thread::sleep_for(1s);
  }

  void resetSimulation() {
    gz::msgs::WorldControl msg;
    msg.mutable_reset()->set_all(true);
    pub.Publish(msg);
    std::this_thread::sleep_for(1s);
  }

  void unpauseSimulation() {
    gz::msgs::WorldControl msg;
    msg.set_pause(false);
    pub.Publish(msg);
  }

  bool deleteEntity(const std::string &name) {
    auto client = this->create_client<ros_gz_interfaces::srv::DeleteEntity>("/world/default/remove");
    if (!client->wait_for_service(3s)) {
      RCLCPP_ERROR(this->get_logger(), "delete_entity service not available.");
      return false;
    }
    auto request = std::make_shared<ros_gz_interfaces::srv::DeleteEntity::Request>();
    request->entity.name = name;
    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to delete entity.");
      return false;
    }
    std::this_thread::sleep_for(1s);
    return true;
  }

  bool spawnEntity(const std::string &name, const std::string &sdf_path, double x, double y) {
    auto client = this->create_client<ros_gz_interfaces::srv::SpawnEntity>("/world/default/create");
    if (!client->wait_for_service(3s)) {
      RCLCPP_ERROR(this->get_logger(), "spawn_entity service not available.");
      return false;
    }

    std::ifstream sdf_file(sdf_path);
    if (!sdf_file) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open SDF file.");
      return false;
    }

    std::string xml((std::istreambuf_iterator<char>(sdf_file)),
                    std::istreambuf_iterator<char>());

    auto request = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
    request->entity_factory.name = name;
    request->entity_factory.sdf = sdf_path_;
    request->entity_factory.pose.position.x = x;
    request->entity_factory.pose.position.y = y;
    request->entity_factory.pose.position.z = 0.0;

    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to spawn entity.");
      return false;
    }
    return true;
  }

  std::shared_ptr<gz::transport::Node> gz_node_;
  gz::transport::Node::Publisher pub;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string robot_name_;
  std::string sdf_path_;
  double initial_x_, initial_y_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ResetAndRespawnNode>());
  rclcpp::shutdown();
  return 0;
}
