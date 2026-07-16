
// sunVisual, Entity ID: 88

#include "SunGlareMovementPlugin.hh"
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/LightCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/Util.hh>
#include <sdf/Light.hh>



void MovingSun::Configure(const gz::sim::Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              gz::sim::EntityComponentManager &_ecm,
              gz::sim::EventManager &_eventMgr
    ){
        if (_sdf->HasElement("light_name")){
            this->lightName = _sdf->Get<std::string>("light_name");
        }

        _ecm.Each<gz::sim::components::Name, gz::sim::components::Light>(
            [&](const gz::sim::Entity &_entity,
            const gz::sim::components::Name *_name,
            const gz::sim::components::Light *) -> bool {
                if (_name->Data() == this->lightName)
                    {
                        this->lightEntity = _entity;
                        return false; // found it, stop iterating
                    }
                return true;
            }
        );

        if (this->lightEntity == gz::sim::kNullEntity){
            std::cerr << "LightAnimator: could not find light [" << this->lightName << "]\n";
        }

    }

void MovingSun::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm){
        if (_info.paused || this->lightEntity == gz::sim::kNullEntity){return;}
    
        double t = std::chrono::duration<double>(_info.simTime).count();
    
        auto direction = gz::math::Vector3d();
        double intensity = 2.5;

        sdf::Light sun;

        // sun.SetDirection(direction);

        sun.SetIntensity(intensity);

    }

// IGNITION_ADD_PLUGIN(LightAnimator,
//     gz::sim::System,
//     LightAnimator::ISystemConfigure,
//     LightAnimator::ISystemPreUpdate)
// IGNITION_ADD_PLUGIN_ALIAS(LightAnimator, "LightAnimator")