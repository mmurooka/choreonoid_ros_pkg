#include "BodyRosItem.h"
#include <cnoid/BodyItem>
#include <cnoid/Link>
#include <cnoid/Sensor>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>

using namespace cnoid;

void BodyRosItem::initialize(ExtensionManager* ext) { 
  static bool initialized = false;
  int argc = 0;
  char** argv;
  if (!ros::isInitialized())
    ros::init(argc, argv, "choreonoid");
  if (!initialized) {
    ext->itemManager().registerClass<BodyRosItem>("BodyRosItem");
    ext->itemManager().addCreationPanel<BodyRosItem>();
    initialized = true;
  }
}

BodyRosItem::BodyRosItem()
  : os(MessageView::instance()->cout())
{
  controllerTarget = NULL;
  has_trajectory_ = false;
  // TODO: We need to change (remap?) bodyName when this item is moved
  // in choreonoid item tree to create clean namespace for each robot
  rosnode_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle(bodyName));
  joint_state_publisher_ = rosnode_->advertise<sensor_msgs::JointState>("joint_states", 1000);
  joint_state_subscriber_ = rosnode_->subscribe("set_joint_trajectory", 1000, &BodyRosItem::callback, this);
}

BodyRosItem::BodyRosItem(const BodyRosItem& org)
  : ControllerItem(org),
    os(MessageView::instance()->cout())
{
  controllerTarget = NULL;
  has_trajectory_ = false;
  rosnode_ = org.rosnode_;
  joint_state_publisher_ = org.joint_state_publisher_;
  joint_state_subscriber_ = org.joint_state_subscriber_;
}

BodyRosItem::~BodyRosItem()
{
  rosnode_->shutdown();
}

ItemPtr BodyRosItem::doDuplicate() const
{
  return new BodyRosItem(*this);
}

bool BodyRosItem::start(Target* target)
{
  controllerTarget = target;
  simulationBody = target->body();
  timeStep_ = target->worldTimeStep();
  controlTime_ = target->currentTime();
  
  return true;
}

bool BodyRosItem::createSensors(BodyPtr body)
{
  const DeviceList<Sensor> sensors(body->devices());
  sensors.makeIdMap(forceSensors_);
  sensors.makeIdMap(gyroSensors_);
  sensors.makeIdMap(accelSensors_);
  
  force_sensor_publishers_.resize(forceSensors_.size());
  for (size_t i=0; i < forceSensors_.size(); ++i) {
    if (Sensor* sensor = forceSensors_.get(i)) {
      force_sensor_publishers_[i] = rosnode_->advertise<geometry_msgs::Wrench>(sensor->name(), 1);
    }
  }
  rate_gyro_sensor_publishers_.resize(gyroSensors_.size());
  for (size_t i=0; i < gyroSensors_.size(); ++i) {
    if (Sensor* sensor = gyroSensors_.get(i)) {
      rate_gyro_sensor_publishers_[i] = rosnode_->advertise<sensor_msgs::Imu>(sensor->name(), 1);
    }
  }
  accel_sensor_publishers_.resize(accelSensors_.size());
  for (size_t i=0; i < accelSensors_.size(); ++i) {
    if (Sensor* sensor = accelSensors_.get(i)) {
      accel_sensor_publishers_[i] = rosnode_->advertise<geometry_msgs::Accel>(sensor->name(), 1);
    }
  }
}

bool BodyRosItem::control()
{
  controlTime_ = controllerTarget->currentTime();

  // publish current joint states
  joint_state_.header.stamp.fromSec(controlTime_);
  joint_state_.name.resize(simulationBody->numJoints());
  joint_state_.position.resize(simulationBody->numJoints());
  joint_state_.velocity.resize(simulationBody->numJoints());
  joint_state_.effort.resize(simulationBody->numJoints());
  for (int i = 0; i < simulationBody->numJoints(); i++) {
    Link* joint = simulationBody->joint(i);
    joint_state_.name[i] = joint->name();
    joint_state_.position[i] = joint->q();
    joint_state_.velocity[i] = joint->dq();
    joint_state_.effort[i] = joint->u();
  }
  joint_state_publisher_.publish(joint_state_);
  
  // apply joint force based on the trajectory message
  if (has_trajectory_ && controlTime_ >= trajectory_start_) {
    if (trajectory_index_ < points_.size()) {
      unsigned int joint_size = points_[trajectory_index_].positions.size();
      for (unsigned int i = 0; i < joint_size; ++i) {
        int j = joint_number_map_[joint_names_[i]];
        Link* joint = simulationBody->joint(j);
        joint->q() = points_[trajectory_index_].positions[i];
      }
      ros::Duration duration(points_[trajectory_index_].time_from_start.sec,
                             points_[trajectory_index_].time_from_start.nsec);
      trajectory_start_ += duration.toSec();
      trajectory_index_++;
    } else {
      has_trajectory_ = false;
    }
  }
  
  ros::spinOnce();
  return true;
}

void BodyRosItem::input()
{
  // TODO: add code to publish sensor info
}

void BodyRosItem::output()
{
  
}

void BodyRosItem::stop()
{
  
}

void BodyRosItem::callback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
  // copy all the trajectory info to the buffer
  unsigned int joint_size = msg->joint_names.size();
  joint_names_.resize(joint_size);
  for (unsigned int i = 0; i < joint_size; ++i) {
    joint_names_[i] = msg->joint_names[i];
  }
  unsigned int point_size = msg->points.size();
  points_.resize(point_size);
  for (unsigned int i = 0; i < point_size; ++i) {
    points_[i].positions.resize(joint_size);
    for (unsigned int j = 0; j < point_size; ++j) {
      points_[i].positions[j] = msg->points[i].positions[j];
    }
  }
  trajectory_start_ = ros::Time(msg->header.stamp.sec,
                                msg->header.stamp.nsec).toSec();
  if (trajectory_start_ < controlTime_)
    trajectory_start_ = controlTime_;
  trajectory_index_ = 0;
  has_trajectory_ = true;
}

void BodyRosItem::onPositionChanged() {
  BodyItem* ownerBodyItem = findOwnerItem<BodyItem>();
  if (ownerBodyItem) {
    BodyPtr body = ownerBodyItem->body();
    createSensors(body);
    if (bodyName != body->name()) {
      bodyName = body->name();
    }
    for (int i = 0; i < body->numJoints(); i++) {
      Link* joint = body->joint(i);
      joint_number_map_[joint->name()] = i;
    }
  } else {
    bodyName.clear();
  }
}