// Copyright 2024 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <mutex>
#include <ignition/common/Util.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/Joint.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>

#include <ignition/math/PID.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include "OmniDrive.hh"

#define WHEEL_NUM 4

using namespace ignition;
using namespace gazebo;
using namespace systems;

// ---------- SDF element names for the four wheel joints ----------
const std::string kSdfElemJointNames[WHEEL_NUM] = {
    "front_left_joint",   // 0
    "front_right_joint",  // 1
    "rear_left_joint",    // 2
    "rear_right_joint"    // 3
};

// ---------- private data ----------
class ignition::gazebo::systems::OmniDrivePrivate
{
public:
    enum WheelIndex
    {
        FRONT_LEFT  = 0,
        FRONT_RIGHT = 1,
        REAR_LEFT   = 2,
        REAR_RIGHT  = 3
    };

public:
    /// \brief cmd_vel callback (runs in ignition-transport thread)
    void OnCmdVel(const ignition::msgs::Twist &_msg);

    /// \brief publish odometry message
    void UpdateOdometry(const ignition::gazebo::UpdateInfo &_info,
                        const ignition::gazebo::EntityComponentManager &_ecm);

    /// \brief mecanum inverse kinematics: chassis velocity → wheel angular velocity
    void ComputeWheelVelocities(double _vx, double _vy, double _omega,
                                double _target[WHEEL_NUM]);

public:
    // ---- transport ----
    transport::Node node;

    // ---- model & links ----
    Model model{kNullEntity};
    std::string chassisLinkName;
    Entity chassisLink{kNullEntity};

    // ---- wheel joints ----
    std::string wheelJointNames[WHEEL_NUM];
    Entity wheelJoints[WHEEL_NUM];

    // ---- geometry ----
    double wheelRadius{0.076};
    double wheelSeparationX{0.286};   // front–rear distance
    double wheelSeparationY{0.284};   // left–right distance

    // ---- wheel velocity PID ----
    ignition::math::PID wheelPids[WHEEL_NUM];
    double wheelTargetVel[WHEEL_NUM] = {0, 0, 0, 0};

    // ---- cmd_vel ----
    msgs::Twist targetVel;
    std::mutex targetVelMutex;

    // ---- odometry ----
    bool initFlag = false;
    bool configured = false;  // true only if Configure() fully succeeded
    std::string odomFrameId;
    std::string odomChildFrameId;
    ignition::math::Pose3d initPose;
    transport::Node::Publisher odomPub;
};

OmniDrive::OmniDrive()
    : dataPtr(std::make_unique<OmniDrivePrivate>())
{
}

void OmniDrive::Configure(const Entity &_entity,
                          const std::shared_ptr<const sdf::Element> &_sdf,
                          EntityComponentManager &_ecm,
                          EventManager & /*_eventMgr*/)
{
    this->dataPtr->model = Model(_entity);
    if (!this->dataPtr->model.Valid(_ecm))
    {
        ignerr << "OmniDrive plugin should be attached to a model entity."
               << std::endl;
        return;
    }

    // ----- chassis link (derived from first wheel joint's parent) -----
    // Model::LinkByName may fail for URDF-loaded models in some Ignition versions.
    // Instead, resolve chassis from the first wheel joint's parent link (like DiffDrive).
    {
        std::string chassisNameFromSdf = _sdf->Get<std::string>("chassis_link");
        this->dataPtr->chassisLinkName = chassisNameFromSdf;  // keep SDF name for odom
        // Try LinkByName first; if that fails, fall back to joint parent.
        this->dataPtr->chassisLink =
            this->dataPtr->model.LinkByName(_ecm, chassisNameFromSdf);
    }
    if (this->dataPtr->chassisLink == kNullEntity)
    {
        ignwarn << "OmniDrive: LinkByName failed, will derive chassis from "
                << "first wheel joint parent." << std::endl;
    }

    // ----- wheel joints -----
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        this->dataPtr->wheelJointNames[i] =
            _sdf->Get<std::string>(kSdfElemJointNames[i]);
        this->dataPtr->wheelJoints[i] =
            this->dataPtr->model.JointByName(
                _ecm, this->dataPtr->wheelJointNames[i]);
        if (this->dataPtr->wheelJoints[i] == kNullEntity)
        {
            ignerr << "OmniDrive: wheel joint ["
                   << this->dataPtr->wheelJointNames[i] << "] not found."
                   << std::endl;
            return;
        }
    }

    // If chassis link not found via LinkByName, derive from joint parent
    if (this->dataPtr->chassisLink == kNullEntity)
    {
        this->dataPtr->chassisLink =
            _ecm.ParentEntity(this->dataPtr->wheelJoints[0]);
        // Keep SDF-provided name for odom frame; actual entity is resolved
    }

    if (this->dataPtr->chassisLink == kNullEntity)
    {
        ignerr << "OmniDrive: could not determine chassis link." << std::endl;
        return;
    }

    ignwarn << "OmniDrive: chassis link resolved (entity="
            << this->dataPtr->chassisLink
            << ", name from SDF=" << this->dataPtr->chassisLinkName << ")"
            << std::endl;

    // ----- geometry (use defaults if not specified) -----
    if (_sdf->HasElement("wheel_radius"))
        this->dataPtr->wheelRadius = _sdf->Get<double>("wheel_radius");
    if (_sdf->HasElement("wheel_separation_x"))
        this->dataPtr->wheelSeparationX = _sdf->Get<double>("wheel_separation_x");
    if (_sdf->HasElement("wheel_separation_y"))
        this->dataPtr->wheelSeparationY = _sdf->Get<double>("wheel_separation_y");

    ignwarn << "OmniDrive geometry: r=" << this->dataPtr->wheelRadius
            << " sep_x=" << this->dataPtr->wheelSeparationX
            << " sep_y=" << this->dataPtr->wheelSeparationY << std::endl;

    // ----- subscribe cmd_vel -----
    std::string topic{"/model/" + this->dataPtr->model.Name(_ecm) + "/cmd_vel"};
    this->dataPtr->node.Subscribe(
        topic, &OmniDrivePrivate::OnCmdVel, this->dataPtr.get());
    ignwarn << "OmniDrive subscribing to [" << topic << "]" << std::endl;

    // ----- odometry publisher -----
    std::string odomTopic{this->dataPtr->model.Name(_ecm) + "/odometry"};
    this->dataPtr->odomPub =
        this->dataPtr->node.Advertise<msgs::Odometry>(odomTopic);
    this->dataPtr->odomFrameId = this->dataPtr->model.Name(_ecm) + "/odom";
    this->dataPtr->odomChildFrameId =
        this->dataPtr->model.Name(_ecm) + "/" +
        ignition::common::replaceAll(
            this->dataPtr->chassisLinkName, "::", "/");

    // ----- init wheel velocity PID (参数从 SDF 读取，无需重新编译) -----
    // (P, I, D, imax, imin, cmdMax, cmdMin, iLimit)
    {
        double pidP      = _sdf->Get<double>("pid_p",       0.02).first;
        double pidI      = _sdf->Get<double>("pid_i",       0.0).first;
        double pidD      = _sdf->Get<double>("pid_d",       0.0005).first;
        double pidCmdMax = _sdf->Get<double>("pid_cmd_max", 0.5).first;
        double pidCmdMin = _sdf->Get<double>("pid_cmd_min", -0.5).first;

        for (int i = 0; i < WHEEL_NUM; i++)
        {
            this->dataPtr->wheelPids[i].Init(
                pidP, pidI, pidD, 0.0, 0.0, pidCmdMax, pidCmdMin, 0.0);
        }

        ignwarn << "OmniDrive PID: P=" << pidP << " I=" << pidI
                << " D=" << pidD << " cmd=[" << pidCmdMin
                << ", " << pidCmdMax << "]" << std::endl;
    }

    ignwarn << "OmniDrive configured successfully." << std::endl;
    this->dataPtr->configured = true;
}


void OmniDrive::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                          ignition::gazebo::EntityComponentManager &_ecm)
{
    IGN_PROFILE("OmniDrive::PreUpdate");

    // nothing to do if paused or Configure failed
    if (_info.paused)
        return;

    if (!this->dataPtr->configured)
        return;

    if (this->dataPtr->chassisLink == kNullEntity)
        return;

    // ----- ensure chassis components exist -----
    if (!_ecm.Component<components::WorldPose>(this->dataPtr->chassisLink))
    {
        _ecm.CreateComponent(this->dataPtr->chassisLink,
                             components::WorldPose());
    }

    // ----- read target velocity (thread-safe) -----
    msgs::Twist targetVel;
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->targetVelMutex);
        targetVel = this->dataPtr->targetVel;
    }

    double vx = targetVel.linear().x();
    double vy = targetVel.linear().y();
    double omega = targetVel.angular().z();

    // ----- mecanum inverse kinematics -----
    double targets[WHEEL_NUM];
    this->dataPtr->ComputeWheelVelocities(vx, vy, omega, targets);

    // ----- PID 速度闭环控制 -----
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        auto jointEntity = this->dataPtr->wheelJoints[i];

        if (jointEntity == kNullEntity)
            continue;

        // 读取当前关节角速度 (rad/s)
        double currentVel = 0.0;
        auto velComp =
            _ecm.Component<components::JointVelocity>(jointEntity);
        if (velComp && !velComp->Data().empty())
        {
            currentVel = velComp->Data().at(0);
        }

        // PID：error = target - current，输出力矩使实际速度跟踪目标速度
        double error = targets[i] - currentVel;
        double forceCmd = this->dataPtr->wheelPids[i].Update(error, _info.dt);

        // ensure JointForceCmd component exists
        auto forceComp =
            _ecm.Component<components::JointForceCmd>(jointEntity);
        if (!forceComp)
        {
            _ecm.CreateComponent(jointEntity,
                                 components::JointForceCmd({0.0}));
        }
        _ecm.SetComponentData<components::JointForceCmd>(
            jointEntity, {forceCmd});

        // store for logging
        this->dataPtr->wheelTargetVel[i] = targets[i];
    }
}

void OmniDrive::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                           const ignition::gazebo::EntityComponentManager &_ecm)
{
    this->dataPtr->UpdateOdometry(_info, _ecm);
}



void OmniDrivePrivate::OnCmdVel(const ignition::msgs::Twist &_msg)
{
    std::lock_guard<std::mutex> lock(this->targetVelMutex);
    this->targetVel = _msg;
}

void OmniDrivePrivate::ComputeWheelVelocities(double _vx, double _vy,
                                              double _omega,
                                              double _target[WHEEL_NUM])
{
    // X-configuration omni wheel inverse kinematics
    //
    // Wheel axes point outward along diagonals (set by URDF joint rpy):
    //   FL axis → ( 0.707,  0.707)  drive direction → (-0.707,  0.707)  rear-left
    //   FR axis → ( 0.707, -0.707)  drive direction → ( 0.707,  0.707)  front-left
    //   RL axis → (-0.707,  0.707)  drive direction → (-0.707, -0.707)  rear-right
    //   RR axis → (-0.707, -0.707)  drive direction → ( 0.707, -0.707)  front-right
    //
    // Positive ω drives robot in the "drive direction" above.
    // ROS convention: x=forward, y=left, ωz=CCW from top
    //
    //   lx = wheel_separation_x / 2,  ly = wheel_separation_y / 2,  L = lx + ly

    double lx = wheelSeparationX / 2.0;
    double ly = wheelSeparationY / 2.0;
    double L  = lx + ly;
    double r  = wheelRadius;

    _target[FRONT_LEFT]  = (+_vx - _vy - L * _omega) / r;
    _target[FRONT_RIGHT] = (-_vx - _vy - L * _omega) / r;
    _target[REAR_LEFT]   = (+_vx + _vy - L * _omega) / r;
    _target[REAR_RIGHT]  = (-_vx + _vy - L * _omega) / r;
}

void OmniDrivePrivate::UpdateOdometry(
    const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
    // Guard against missing components (e.g. first frame before physics
    // initialises velocities, or simulation in paused state).
    const auto *poseComp =
        _ecm.Component<components::WorldPose>(this->chassisLink);
    const auto *linVelComp =
        _ecm.Component<components::LinearVelocity>(this->chassisLink);
    const auto *angVelComp =
        _ecm.Component<components::AngularVelocity>(this->chassisLink);
    if (!poseComp || !linVelComp || !angVelComp)
        return;

    Link chassisLinkEnt(this->chassisLink);
    const auto chassisPose = poseComp->Data();
    const auto linearVel   = linVelComp->Data();
    const auto angularVel  = angVelComp->Data();

    if (!this->initFlag)
    {
        this->initPose = chassisPose;
        this->initFlag = true;
    }

    // build odometry message
    msgs::Odometry msg;

    // pose
    msg.mutable_pose()->mutable_position()->set_x(chassisPose.X());
    msg.mutable_pose()->mutable_position()->set_y(chassisPose.Y());
    math::Quaterniond orientation(0, 0, chassisPose.Yaw());
    msgs::Set(msg.mutable_pose()->mutable_orientation(), orientation);

    // twist
    msg.mutable_twist()->mutable_linear()->set_x(linearVel.X());
    msg.mutable_twist()->mutable_linear()->set_y(linearVel.Y());
    msg.mutable_twist()->mutable_angular()->set_z(angularVel.Z());

    // header
    msg.mutable_header()->mutable_stamp()->CopyFrom(
        convert<msgs::Time>(_info.simTime));
    auto frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(this->odomFrameId);
    auto childFrame = msg.mutable_header()->add_data();
    childFrame->set_key("child_frame_id");
    childFrame->add_value(this->odomChildFrameId);

    this->odomPub.Publish(msg);
}

IGNITION_ADD_PLUGIN(OmniDrive,
                    ignition::gazebo::System,
                    OmniDrive::ISystemConfigure,
                    OmniDrive::ISystemPreUpdate,
                    OmniDrive::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(OmniDrive, "ignition::gazebo::systems::OmniDrive")
