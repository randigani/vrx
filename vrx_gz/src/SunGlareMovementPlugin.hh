
#ifndef SUN_GLARE_MOVEMENT_PLUGIN_HH_
#define SUN_GLARE_MOVEMEMT_PLUGIN_HH_

// sunVisual, Entity ID: 88

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>

namespace custom{

    class MovingSun : 
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
    {
        public:
            void Configure(const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventMgr) override;

            void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

        private:
            
            gz::sim::Entity lightEntity{gz::sim::kNullEntity};
            std::string lightName;

            double s_headstart;
            double solstice;    // -1 ~ 1. Changes max sun height +/- 23.5 degrees. s
            double lat_rad;
            double half_d;

            double lastRecreateTime = -1000.0;
    };
}


#endif
