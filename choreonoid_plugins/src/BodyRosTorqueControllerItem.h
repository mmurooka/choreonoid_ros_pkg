/**
  @file
  @author
 */

#ifndef CNOID_ROS_PLUGIN_BODY_ROS_TORQUE_CONTROLLER_ITEM_H_INCLUDED
#define CNOID_ROS_PLUGIN_BODY_ROS_TORQUE_CONTROLLER_ITEM_H_INCLUDED

#include "BodyRosJointControllerItem.h"

#include <cnoid/PutPropertyFunction>
#include <cnoid/YAMLReader>
#include <cnoid/FileUtil>

namespace cnoid {

class CNOID_EXPORT BodyRosTorqueControllerItem : public BodyRosJointControllerItem
{
public:
    static void initialize(ExtensionManager* ext);
    
    BodyRosTorqueControllerItem();
    BodyRosTorqueControllerItem(const BodyRosTorqueControllerItem& org);
    virtual ~BodyRosTorqueControllerItem();

protected:
    virtual Item* doDuplicate() const;
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
    void doPutProperties(PutPropertyFunction& putProperty);

    virtual bool hook_of_start();
    virtual void keep_attitude();
    virtual void apply_message(Link* joint, size_t idx, trajectory_msgs::JointTrajectoryPoint* point);
    virtual bool copy_message(
                  const trajectory_msgs::JointTrajectoryPoint* msg,
                  trajectory_msgs::JointTrajectoryPoint* dst,
                  size_t idx,
                  unsigned int jsz
                  );

private:
    std::string pdc_parameter_filename_;
    std::vector<double> pgain;
    std::vector<double> dgain;
    std::vector<double> u_lower;
    std::vector<double> u_upper;

    std::vector<double> qref_old_;
    std::vector<double> q_old_;

    /**
     */
    bool set_pdc_parameters(Listing* src, std::vector<double>& dst);

    /**
     */
    bool load_pdc_parameters();

    /**
     */
    void pd_control(Link* joint, double q_ref);
};

typedef ref_ptr<BodyRosTorqueControllerItem> BodyRosTorqueControllerItemPtr;
}

#endif  /* CNOID_ROS_PLUGIN_BODY_ROS_TORQUE_CONTROLLER_ITEM_H_INCLUDED */
