#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Actor.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Actor.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/plugin/Register.hh>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>

#include <mutex>
#include <vector>

using namespace ignition;
using namespace gazebo;

class ActorTrajectoryPlugin
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate
{
public:
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &) override
  {
    this->entity = _entity;
    this->actorName = _sdf->Get<std::string>("actor_name", "actor_walking").first;
    this->rosTopic = _sdf->Get<std::string>("ros_topic", "/person_path").first;

    // Init ROS2 node
    rclcpp::init(0, nullptr);
    this->rosNode = std::make_shared<rclcpp::Node>("actor_trajectory_plugin");

    using std::placeholders::_1;
    this->subPath = this->rosNode->create_subscription<nav_msgs::msg::Path>(
        this->rosTopic, 10, std::bind(&ActorTrajectoryPlugin::PathCallback, this, _1));

    RCLCPP_INFO(this->rosNode->get_logger(), "ActorTrajectoryPlugin started for actor [%s], topic [%s]",
                this->actorName.c_str(), this->rosTopic.c_str());
  }

  void PreUpdate(const UpdateInfo &_info,
                 EntityComponentManager &_ecm) override
  {
    if (!this->rosNode)
      return;

    rclcpp::spin_some(this->rosNode);

    std::lock_guard<std::mutex> lock(this->mutex);
    if (this->pathPoses.empty())
      return;

    // Simzeit in Sekunden
    double simTime = std::chrono::duration<double>(_info.simTime).count();

    // Nächste Pose aus der Path-Liste interpolieren
    geometry_msgs::msg::Pose pose = InterpolatePose(simTime);

    // SetTrajectoryPose am Actor setzen
    auto actorEntity = _ecm.EntityByComponents(components::Name(this->actorName));
    if (actorEntity != kNullEntity)
    {
      auto gzPose = math::Pose3d(
          pose.position.x, pose.position.y, pose.position.z,
          pose.orientation.w, pose.orientation.x,
          pose.orientation.y, pose.orientation.z);
      _ecm.SetComponentData<components::Pose>(actorEntity, gzPose);
    }
  }

private:
  void PathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->pathPoses = msg->poses;
  }

  geometry_msgs::msg::Pose InterpolatePose(double simTime)
  {
    if (this->pathPoses.size() == 1)
      return this->pathPoses.front().pose;

    // Suche den richtigen Zeitbereich
    for (size_t i = 0; i < this->pathPoses.size() - 1; ++i)
    {
      double t0 = rclcpp::Time(this->pathPoses[i].header.stamp).seconds();
      double t1 = rclcpp::Time(this->pathPoses[i+1].header.stamp).seconds();

      if (simTime >= t0 && simTime <= t1)
      {
        double alpha = (simTime - t0) / (t1 - t0);
        geometry_msgs::msg::Pose p0 = this->pathPoses[i].pose;
        geometry_msgs::msg::Pose p1 = this->pathPoses[i+1].pose;

        geometry_msgs::msg::Pose p;
        p.position.x = p0.position.x + alpha * (p1.position.x - p0.position.x);
        p.position.y = p0.position.y + alpha * (p1.position.y - p0.position.y);
        p.position.z = p0.position.z + alpha * (p1.position.z - p0.position.z);
        p.orientation = p0.orientation; // einfach übernommen; slerp möglich
        return p;
      }
    }
    return this->pathPoses.back().pose;
  }

private:
  Entity entity{kNullEntity};
  std::string actorName;
  std::string rosTopic;

  std::shared_ptr<rclcpp::Node> rosNode;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath;

  std::mutex mutex;
  std::vector<geometry_msgs::msg::PoseStamped> pathPoses;
};

IGNITION_ADD_PLUGIN(ActorTrajectoryPlugin,
              ignition::gazebo::System,
              ActorTrajectoryPlugin::ISystemConfigure,
              ActorTrajectoryPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ActorTrajectoryPlugin, "actor_trajectory_plugin")

