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

#ifndef IGNITION_GAZEBO_SYSTEMS_OMNI_DRIVE_HH
#define IGNITION_GAZEBO_SYSTEMS_OMNI_DRIVE_HH

#include <memory>
#include <ignition/gazebo/System.hh>

namespace ignition
{
    namespace gazebo
    {
        namespace systems
        {
            class OmniDrivePrivate;

            /// \brief Mecanum omnidirectional drive plugin.
            ///
            /// Subscribes to /<model>/cmd_vel (ignition::msgs::Twist),
            /// computes target angular velocity for each wheel using mecanum
            /// inverse kinematics, and commands joints via velocity PID.
            ///
            /// SDF parameters:
            ///   <chassis_link>        - chassis link name
            ///   <front_left_joint>    - front-left wheel joint name
            ///   <front_right_joint>   - front-right wheel joint name
            ///   <rear_left_joint>     - rear-left wheel joint name
            ///   <rear_right_joint>    - rear-right wheel joint name
            ///   <wheel_radius>        - wheel radius (m)
            ///   <wheel_separation_x>  - front-rear wheel distance (m)
            ///   <wheel_separation_y>  - left-right wheel distance (m)
            class IGNITION_GAZEBO_VISIBLE OmniDrive
                : public ignition::gazebo::System,
                  public ISystemConfigure,
                  public ISystemPreUpdate,
                  public ISystemPostUpdate
            {
            public:
                OmniDrive();
                ~OmniDrive() override = default;

            public:
                void Configure(const Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               EntityComponentManager &_ecm,
                               EventManager &_eventMgr) override;

                void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                               ignition::gazebo::EntityComponentManager &_ecm) override;

                void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                                const ignition::gazebo::EntityComponentManager &_ecm) override;

            private:
                std::unique_ptr<OmniDrivePrivate> dataPtr;
            };

        }  // namespace systems
    }  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_SYSTEMS_OMNI_DRIVE_HH
