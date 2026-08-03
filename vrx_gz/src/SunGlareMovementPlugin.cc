
// Entity ID: 87

#include "SunGlareMovementPlugin.hh"
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/LightCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/CastShadows.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <sdf/Light.hh>

using namespace custom;

constexpr double tilt_rad = 23.44 * M_PI / 180;
// constexpr double lat_rad = 40.7128 * M_PI / 180; // NYC latitude
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

        half_d = [&_sdf](){
            uint rv = 43200;

            if (const char *_env_var = std::getenv("SUN_CYCLE_PERIOD")){
                try {
                    rv = static_cast<uint32_t>(std::stoul(_env_var));
                    gzmsg << "SunGlareMovementPlugin: Using env variable SUN_CYCLE_PERIOD=" << rv << "\n";

                } catch (const std::exception &e){
                    gzwarn << "SunGlareMovementPlugin: SUN_CYCLE_PERIOD env var '" << _env_var
                        << "' is not a valid uint (" << e.what() << ")\n";
                }

            } else if (_sdf->HasElement("sun_cycle_period")){
                rv = _sdf->Get<uint>("sun_cycle_period", rv).first;
                gzmsg << "SunGlareMovementPlugin: Using SDF (or SDF default) sun_cycle_period=" << rv << "\n";
            
            } else {
                gzmsg << "SunGlareMovementPlugin: sun_cycle_period was not provided in sdf, nor was it found as an env var. Setting default value: "
                    << rv << "\n";
            }

            if (rv < 3){
                rv = 43200;
                gzwarn << "SunGlareMovementPlugin: sun_cycle_period can't be less than 3s! Setting default value: " << rv << "\n";
            } else if (rv < 30){
                gzwarn << "SunGlareMovementPlugin: sun_cycle_period is very short (less than 30 secs): " << rv
                    << ". Accepting, but simulation might look ridiculous!\n";
            }

            return static_cast<double>(rv);
        }();

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

        // Extension to SunGlareMovement: Add a massive color filter sphere in front of the skybox, so when the sun lowers it looks like evening and night.
        // The filter color should depend on the sun's elevation. 
        // Sun above 12 degrees: Normal
        // Sun above 6 degrees: Subtle yellowish
        // Sun above 0 degrees: Yellow-orange
        // Sun above -4 degrees: Orange-red
        // Sun below -6 degrees: Dark blue
        // Sun below -12 degrees: Black 

    }

GZ_ADD_PLUGIN(MovingSun,
    gz::sim::System,
    MovingSun::ISystemConfigure,
    MovingSun::ISystemPreUpdate);
GZ_ADD_PLUGIN_ALIAS(MovingSun, "custom::MovingSun");