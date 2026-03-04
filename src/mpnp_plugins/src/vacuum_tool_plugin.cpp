#include "mpnp_plugins/vacuum_tool_plugin.hpp"

GZ_ADD_PLUGIN(
  mpnp_plugins::VacuumToolPlugin,
  gz::sim::System,
  mpnp_plugins::VacuumToolPlugin::ISystemConfigure,
  mpnp_plugins::VacuumToolPlugin::ISystemPreUpdate
)

using namespace mpnp_plugins;

VacuumToolPlugin::~VacuumToolPlugin()
{
  executor->cancel();
  thread_executor_spin.join();
}

void VacuumToolPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  // Read SDF tags
  std::string ros_namespace = "";
  if (_sdf->HasElement("ros")){
    if (_sdf->GetElementImpl("ros")->HasElement("namespace")){
      ros_namespace = _sdf->GetElementImpl("ros")->Get<std::string>("namespace");
    }
  }

  tool_type = _sdf->Get<int>("tool_type");

  // GZ setup
  auto model = gz::sim::Model(_entity);
  gripper_base_link = model.LinkByName(_ecm, "onrobot_vgc10_base_link");

  gz_node = std::make_shared<gz::transport::Node>();

  std::vector<std::string> topic_names; 
  std::string topic = model.Name(_ecm) + "/suction{n}/contact_sensor";

  int suction_cup_count = tool_type == VacuumTools::VG_2 ? 2 : 4;
  for(int i = 1; i <= suction_cup_count; i++){
    std::string name = topic;
    topic_names.push_back(name.replace(topic.find("{n}"), 3, std::to_string(i)));
  }

  gz_node->Subscribe(topic_names[0], &VacuumToolPlugin::contact_sensor_1_cb, this);
  gz_node->Subscribe(topic_names[1], &VacuumToolPlugin::contact_sensor_2_cb, this);
  suction_cup_joints.push_back(gz::sim::Joint(gz::sim::Model(model).JointByName(_ecm, "suction1_joint")));
  suction_cup_joints.push_back(gz::sim::Joint(gz::sim::Model(model).JointByName(_ecm, "suction2_joint")));

  if(tool_type == VacuumTools::VG_4) {
    gz_node->Subscribe(topic_names[2], &VacuumToolPlugin::contact_sensor_3_cb, this);
    gz_node->Subscribe(topic_names[3], &VacuumToolPlugin::contact_sensor_4_cb, this);
    suction_cup_joints.push_back(gz::sim::Joint(gz::sim::Model(model).JointByName(_ecm, "suction3_joint")));
    suction_cup_joints.push_back(gz::sim::Joint(gz::sim::Model(model).JointByName(_ecm, "suction4_joint")));
  }

  // ROS setup
  if (!rclcpp::ok()){
    rclcpp::init(0, nullptr);
  }

  ros_node = rclcpp::Node::make_shared(model.Name(_ecm) + "_plugin_node", ros_namespace);

  rclcpp::Parameter sim_time("use_sim_time", true);
  ros_node->set_parameter(sim_time);

  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(ros_node);

  auto spin = [this](){
    while(rclcpp::ok()){
      executor->spin_once();
    }
  };

  thread_executor_spin = std::thread(spin);

  if (tool_type == VacuumTools::VG_2) {
    attach_srv = ros_node->create_service<Trigger>(
      "grasp",
      std::bind(&VacuumToolPlugin::vg_2_attach_cb, this, std::placeholders::_1, std::placeholders::_2)
    );
  } else if(tool_type == VacuumTools::VG_4) {
    attach_srv = ros_node->create_service<Trigger>(
      "grasp",
      std::bind(&VacuumToolPlugin::vg_4_attach_cb, this, std::placeholders::_1, std::placeholders::_2)
    );
  } else {
    gzerr << "Invalid tool type specified in SDF. Supported types are 2 (VG-2) and 4 (VG-4)." << std::endl;
  }

  detach_srv = ros_node->create_service<Trigger>(
    "release",
    std::bind(&VacuumToolPlugin::detach_object_cb, this, std::placeholders::_1, std::placeholders::_2)
  );
}

void VacuumToolPlugin::PreUpdate(const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager &_ecm)
{
  switch (lock_state)
  {
  case VacuumToolLockState::UNLOCKED: {
    for(gz::sim::Joint joint : suction_cup_joints){
      joint.SetVelocity(_ecm, {-0.0008});
    }
    break;
  }
  case VacuumToolLockState::LOCK_REQUESTED: {
    // Get link entity for bottom obj
    auto model = _ecm.EntityByName(attach_obj_name);
    
    if (!model.has_value()) {
      gzerr << "Unable to locate obj model: " << attach_obj_name << std::endl;
      lock_state = VacuumToolLockState::UNLOCKED;
      break;
    }

    auto obj_link = gz::sim::Model(model.value()).LinkByName(_ecm, "base_link");

    // Lock joint
    lock_joint = _ecm.CreateEntity();

    _ecm.CreateComponent(lock_joint, gz::sim::components::DetachableJoint({gripper_base_link, obj_link, "fixed"}));

    lock_state = VacuumToolLockState::LOCKED;

    break;
  }
  case VacuumToolLockState::UNLOCK_REQUESTED:
    // Unlock joint
    _ecm.RequestRemoveEntity(lock_joint);
    lock_joint = gz::sim::kNullEntity;

    lock_state = VacuumToolLockState::UNLOCKED;
    for(gz::sim::Joint joint : suction_cup_joints){
      joint.ResetPosition(_ecm, {0.0});
    }
    break;

  case VacuumToolLockState::LOCKED:
    // Do nothing
    break;

  default:
    gzerr << "Unknown lock state\n";
    break;
  }
}

void VacuumToolPlugin::vg_2_attach_cb(const TriggerReqPtr request, TriggerResPtr response)
{
  attach_obj_name = request->target_obj;

  if(lock_state == VacuumToolLockState::LOCKED){
    response->success = false;
    response->message = "Already holding object";
    return;
  }

  if (!pad_contacts[1].in_contact && !pad_contacts[2].in_contact) {
    response->success = false;
    response->message = "Suction cups must be in contact with an obj";
    return;
  }

  std::string target_obj_name = pad_contacts[1].model_name;

  if (strcmp(target_obj_name.c_str(), attach_obj_name.c_str()) != 0) {
    response->success = false;
    response->message = "Target object not in contact with suction cups";
    return;
  }

  lock_state = VacuumToolLockState::LOCK_REQUESTED;

  response->success = wait_for_state(VacuumToolLockState::LOCKED);
  response->message = response->success ? "Top obj attached" : "Unable to grasp object";
}

void VacuumToolPlugin::vg_4_attach_cb(const TriggerReqPtr request, TriggerResPtr response)
{
  attach_obj_name = request->target_obj;

  if(lock_state == VacuumToolLockState::LOCKED){
    response->success = false;
    response->message = "Already holding object";
    return;
  }
  
  if ((!pad_contacts[1].in_contact && !pad_contacts[2].in_contact) || (!pad_contacts[3].in_contact && !pad_contacts[4].in_contact)) {
    response->success = false;
    response->message = "Suction cups must be in contact with an obj";
    return;
  }

  std::string target_obj_name = pad_contacts[1].model_name;

  if (strcmp(target_obj_name.c_str(), attach_obj_name.c_str()) != 0) {
    response->success = false;
    response->message = "Target object " + target_obj_name + " not the same as attach object " + attach_obj_name;
    return;
  }

  lock_state = VacuumToolLockState::LOCK_REQUESTED;

  response->success = wait_for_state(VacuumToolLockState::LOCKED);
  response->message = response->success ? "Module attached" : "Unable to grasp object";
}

void VacuumToolPlugin::detach_object_cb(const TriggerReqPtr request, TriggerResPtr response)
{
  if(lock_state != VacuumToolLockState::LOCKED){
    response->success = false;
    response->message = "Tool not holding object";
    return;
  }

  lock_state = VacuumToolLockState::UNLOCK_REQUESTED;

  response->success = wait_for_state(VacuumToolLockState::UNLOCKED);
  response->message = response->success ? "Object detached" : "Unable to release object";
}

void VacuumToolPlugin::contact_sensor_1_cb(const gz::msgs::StringMsg_V &msg)
{
  auto obj = obj_in_contact(msg);

  pad_contacts[1] = PadContact{obj.has_value(), obj.has_value() ? obj.value() : ""};
}

void VacuumToolPlugin::contact_sensor_2_cb(const gz::msgs::StringMsg_V &msg)
{
  auto obj = obj_in_contact(msg);

  pad_contacts[2] = PadContact{obj.has_value(), obj.has_value() ? obj.value() : ""};
}

void VacuumToolPlugin::contact_sensor_3_cb(const gz::msgs::StringMsg_V &msg)
{
  auto obj = obj_in_contact(msg);

  pad_contacts[3] = PadContact{obj.has_value(), obj.has_value() ? obj.value() : ""};
}

void VacuumToolPlugin::contact_sensor_4_cb(const gz::msgs::StringMsg_V &msg)
{
  auto obj = obj_in_contact(msg);

  pad_contacts[4] = PadContact{obj.has_value(), obj.has_value() ? obj.value() : ""};
}

std::optional<std::string> VacuumToolPlugin::obj_in_contact(const gz::msgs::StringMsg_V &msg)
{
  auto data = msg.data();

  if(data.empty()){
    return std::nullopt;
  }

  return data.at(0);
}

bool VacuumToolPlugin::wait_for_state(VacuumToolLockState state)
{
  rclcpp::Time start_time = ros_node->now();
  rclcpp::Rate rate(100);
  while(rclcpp::ok()){
    if (lock_state == state) {
      break;
    } else if (ros_node->now() - start_time > rclcpp::Duration::from_seconds(3.0)) {
      gzerr << "Timed out during request\n";
      return false;
    }
    rate.sleep();
  };

  return lock_state == state;
}
