// Copyright 2025 RoboMaster-OSS
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
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/Model.hh>

#include "JointForceController.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

// ---------- private data ----------
class ignition::gazebo::systems::JointForceControllerPrivate
{
public:
    /// \brief cmd_force callback (runs in ignition-transport thread)
    void OnCmdForce(const ignition::msgs::Double &_msg);

public:
    transport::Node node;
    Model model{kNullEntity};
    std::string jointName;
    Entity jointEntity{kNullEntity};

    double cmdForce{0.0};
    std::mutex cmdForceMutex;
};

void JointForceControllerPrivate::OnCmdForce(const ignition::msgs::Double &_msg)
{
    std::lock_guard<std::mutex> lock(this->cmdForceMutex);
    this->cmdForce = _msg.data();
}

// ====================================================================

JointForceController::JointForceController()
    : dataPtr(std::make_unique<JointForceControllerPrivate>())
{
}

void JointForceController::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager & /*_eventMgr*/)
{
    this->dataPtr->model = Model(_entity);
    if (!this->dataPtr->model.Valid(_ecm))
    {
        ignerr << "JointForceController plugin should be attached to a model entity."
               << std::endl;
        return;
    }

    // ----- joint name -----
    if (!_sdf->HasElement("joint_name"))
    {
        ignerr << "JointForceController: missing <joint_name> element." << std::endl;
        return;
    }
    this->dataPtr->jointName = _sdf->Get<std::string>("joint_name");
    this->dataPtr->jointEntity =
        this->dataPtr->model.JointByName(_ecm, this->dataPtr->jointName);
    if (this->dataPtr->jointEntity == kNullEntity)
    {
        ignerr << "JointForceController: joint [" << this->dataPtr->jointName
               << "] not found." << std::endl;
        return;
    }

    // ----- subscribe cmd_force -----
    std::string topic = "/model/" +
                        this->dataPtr->model.Name(_ecm) +
                        "/joint/" + this->dataPtr->jointName +
                        "/cmd_force";
    this->dataPtr->node.Subscribe(
        topic, &JointForceControllerPrivate::OnCmdForce, this->dataPtr.get());
    ignmsg << "JointForceController subscribing to [" << topic << "]" << std::endl;
}

void JointForceController::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
    if (_info.paused)
        return;

    // ----- read target force (thread-safe) -----
    double cmdForce;
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->cmdForceMutex);
        cmdForce = this->dataPtr->cmdForce;
    }

    // ----- ensure JointForceCmd component exists -----
    auto forceComp =
        _ecm.Component<components::JointForceCmd>(this->dataPtr->jointEntity);
    if (!forceComp)
    {
        _ecm.CreateComponent(this->dataPtr->jointEntity,
                             components::JointForceCmd({0.0}));
    }
    _ecm.SetComponentData<components::JointForceCmd>(
        this->dataPtr->jointEntity, {cmdForce});
}

IGNITION_ADD_PLUGIN(JointForceController,
                    ignition::gazebo::System,
                    JointForceController::ISystemConfigure,
                    JointForceController::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(JointForceController,
                          "ignition::gazebo::systems::JointForceController")
