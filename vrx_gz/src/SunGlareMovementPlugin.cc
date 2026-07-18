
// Entity ID: 87

#include "SunGlareMovementPlugin.hh"
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/LightCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/CastShadows.hh>
#include <gz/sim/Util.hh>
#include <sdf/Light.hh>

using namespace custom;

constexpr double tilt_rad = 23.44 * M_PI / 180;
// constexpr double lat_rad = 40.7128 * M_PI / 180; // NYC latitude
constexpr int half_d = 43200;
constexpr double recreateInterval = 3.0;


void MovingSun::Configure(const gz::sim::Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              gz::sim::EntityComponentManager &_ecm,
              gz::sim::EventManager &_eventMgr
    ){

        s_headstart = _sdf->Get<double>("start_time", 0).first;
        solstice = _sdf->Get<double>("solstice", 0).first;
        lightName = _sdf->Get<std::string>("light_name", "sun").first;
        lat_rad = (_sdf->Get<double>("latitude", 40.7128).first) * M_PI / 180;

    }

void MovingSun::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm){

        if (this->lightEntity == gz::sim::kNullEntity){
            _ecm.Each<gz::sim::components::Name, gz::sim::components::Light>(
                [&](const gz::sim::Entity &_entity,
                    const gz::sim::components::Name *_name,
                    const gz::sim::components::Light *) -> bool
                {
                if (_name->Data() == this->lightName)
                {
                    this->lightEntity = _entity;
                    gzmsg << "MovingSun: found light entity " << _entity << "\n";
                    return false;
                }
                return true;
                });

            if (this->lightEntity == gz::sim::kNullEntity)
            return; // not created yet, try again next tick
        }

        if (_info.paused){return;}
    
        double t = std::chrono::duration<double>(_info.simTime).count();

        if (t - this->lastRecreateTime < recreateInterval)
            {return;}
        
        lastRecreateTime = t;
    
        // Sun movement is approximated by a sine function. 
        // If the sun moves perfectly from east to west, the sun takes 43200 secs and at max height is 66.5 degrees.

        auto lightComp = _ecm.Component<gz::sim::components::Light>(this->lightEntity);
        if (!lightComp)
            {return;}


        double decl_rad = solstice * tilt_rad;

        double hour_angle = M_PI * ((t + s_headstart) - (half_d / 2)) / half_d;

        double sin_elev = (sin(lat_rad) * sin(decl_rad)) + (cos(lat_rad) * cos(decl_rad) * cos(hour_angle));
        double elev = asin(sin_elev);

        double cos_az = (std::sin(decl_rad) - (std::sin(lat_rad) * std::sin(elev))) / (std::cos(lat_rad) * std::cos(elev));
        cos_az = std::fmax(-1.0, std::fmin(1.0, cos_az));
        double azimuth = acos(cos_az);

        if (hour_angle > 0.0) {
            azimuth = (2.0 * M_PI) - azimuth;
        }

        if (elev < 0){elev = 0;}

        // x' = cos(pi * t / 43200), y' = sin(pi * t / 43200)

        auto sun_loc = gz::math::v7::Vector3d(
            cos(elev) * sin(azimuth),
            cos(elev) * cos(azimuth),
            sin(elev)
        );

        auto direction = -sun_loc; // Vector back to the origin is just the negative position vector

        // auto lightComp = _ecm.Component<gz::sim::components::Light>(this->lightEntity);
        sdf::Light sun = lightComp->Data();
        sun.SetDirection(direction);

        auto parentComp = _ecm.Component<gz::sim::components::ParentEntity>(this->lightEntity);
        gz::sim::Entity worldEntity = parentComp->Data();

        // gzmsg << direction.X();
        // gzmsg << direction.Y();
        // gzmsg << direction.Z() << "\n";

        auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->lightEntity);
        gz::math::Pose3d lightPose = poseComp ? poseComp->Data() : gz::math::Pose3d::Zero;

        bool castShadows = sun.CastShadows();

        _ecm.RequestRemoveEntity(this->lightEntity);

        auto newEntity = _ecm.CreateEntity();
        _ecm.CreateComponent(newEntity, gz::sim::components::Light(sun));
        _ecm.CreateComponent(newEntity, gz::sim::components::Name(this->lightName));
        _ecm.CreateComponent(newEntity, gz::sim::components::Pose(lightPose));
        _ecm.CreateComponent(newEntity, gz::sim::components::ParentEntity(worldEntity));
        _ecm.CreateComponent(newEntity, gz::sim::components::CastShadows(castShadows));

        this->lightEntity = newEntity;

    }

GZ_ADD_PLUGIN(MovingSun,
    gz::sim::System,
    MovingSun::ISystemConfigure,
    MovingSun::ISystemPreUpdate);
GZ_ADD_PLUGIN_ALIAS(MovingSun, "custom::MovingSun");