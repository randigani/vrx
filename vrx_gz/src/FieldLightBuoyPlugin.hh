/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef VRX_FIELD_LIGHT_BUOY_PLUGIN_HH_
#define VRX_FIELD_LIGHT_BUOY_PLUGIN_HH_

#include <memory>
#include <sdf/sdf.hh>
#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>

namespace vrx
{
  /// \brief Plugin for setting buoy colors based on underlying scalar field.
  ///
  /// This plugin implements the Yu et al. (2016) "Correlated Orienteering Problem"
  /// approach where buoy colors represent spatially correlated scalar field values
  /// that vary over time.
  ///
  /// Supports all environments: uniform_distrib_env, clustered_distrib_env,
  /// boundary_distrib_env. The environment is specified per-buoy in the SDF.
  ///
  /// SDF Parameters:
  /// <environment>: Environment type (required)
  /// <update_interval>: Seconds between color updates (default: 12.0)
  /// <visuals>: The collection of visuals that change in color. It accepts N
  ///            elements of <visual> elements.
  ///  
  /// The plugin should be inside a <visual> element.
  /// Example:
  ///   <plugin name="vrx::FieldLightBuoyPlugin" filename="libFieldLightBuoyPlugin.so">
  ///     <environment>clustered_distrib_env</environment>
  ///     <update_interval>12.0</update_interval>
  ///     <visuals>
  ///       <visual>robotx_light_buoy::base_link::panel_1</visual>
  ///       <visual>robotx_light_buoy::base_link::panel_2</visual>
  ///       <visual>robotx_light_buoy::base_link::panel_3</visual>
  ///     </visuals>
  ///   </plugin>
  class FieldLightBuoyPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
  {
    /// \brief Constructor
    public: FieldLightBuoyPlugin();

    /// \brief Destructor.
    public: ~FieldLightBuoyPlugin() override = default;

    // Documentation inherited.
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}
#endif