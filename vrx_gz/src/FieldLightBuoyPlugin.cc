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

#include <gz/msgs/color.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <iomanip>
#include <map>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Visual.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

#include <sdf/sdf.hh>

#include "FieldLightBuoyPlugin.hh"

using namespace gz;
using namespace vrx;

struct GaussianProcessParams
{
  double centerX;
  double centerY;
  double spatialLengthScale;      // l_s: spatial correlation length
  double temporalLengthScale;     // l_t: temporal correlation scale
  double variance;                // sigma^2: process variance
  double meanValue;               // mu: mean field value
  double noiseStdDev;             // sigma_noise: observation noise

  // Ornstein-Uhlenbeck process parameters for temporal evolution
  double meanReversionRate;       // theta: speed of reversion to mean
  double diffusionCoeff;          // sigma_diff: random fluctuation strength

  // Current field state (for temporal correlation)
  double currentAmplitude;
};

// ============================================================================
// STATIC GP STATE
static std::mutex s_gpMutex;
static bool s_gpInitialized = false;
static std::string s_gpEnvironment;
static std::vector<GaussianProcessParams> s_sharedGPs;
static std::mt19937 s_sharedRng;
static std::normal_distribution<double> s_sharedNormalDist{0.0, 1.0};
static double s_lastGPUpdateTimeSec = -1.0;
static double s_lastColorPubTimeSec = -1.0;

static gz::transport::Node s_transportNode;
static gz::transport::Node::Publisher s_gpPub;
static bool s_publisherCreated = false;

struct BuoyCachedState
{
  uint64_t entityId;
  double x;
  double y;
  double fieldValue;
  std::string color;
};
static std::map<uint64_t, BuoyCachedState> s_buoyCache;

static int s_totalBuoyInstances = 0;
static int s_buoysEvaluatedThisCycle = 0;

static gz::transport::Node::Publisher s_colorPub;
static bool s_isComputeOwner = false;
static bool s_ownershipDetermined = false;

static std::mutex s_rxColorMutex;
static std::map<uint64_t, std::string> s_receivedColors;
static bool s_subscribedToColors = false;

// ============================================================================

class FieldLightBuoyPlugin::Implementation
{
  public: static msgs::Color CreateColor(const double _r, const double _g,
                                         const double _b, const double _a);
  public: bool ParseSDF(sdf::ElementPtr _sdf);
  public: void InitializeField(const std::string &_environment);
  public: double EvaluateScalarField(double _x, double _y, double _t);
  public: double EvaluateGaussianProcessField(double _x, double _y, double _t);
  public: double EvaluateGradientField(double _x, double _y, double _t);
  public: std::string FieldValueToColor(double _fieldValue);
  public: void Update();
  public: bool GetBuoyPosition(double &_x, double &_y);
  public: static void UpdateSharedGaussianProcesses(double _dt);
  public: static void PublishGPState(double _simTime);
  public: static void PublishBuoyColors();
  public: static double SharedSampleGaussian(double _mean, double _stddev);
  public: static void OnBuoyColors(const gz::msgs::StringMsg &_msg);

  public: static std::map<std::string, gz::msgs::Color> kColors;
  public: std::vector<std::string> visualNames;
  public: std::vector<gz::rendering::VisualPtr> visuals;
  public: rendering::ScenePtr scene;
  public: std::string environment;
  public: std::string fieldType;

  public: double baseValue;
  public: double minValue;
  public: double maxValue;
  public: double xRangeMin;
  public: double xRangeMax;
  public: double gradientAmplitudeBase;
  public: double gradientAmplitudeRange;
  public: double gradientPeriod;
  public: double noiseLevel;

  public: std::vector<double> colorThresholds = {0.0, 0.3, 0.6, 0.9, 1.5};
  public: std::vector<std::string> colorNames = {"blue", "green", "yellow", "red"};
  public: double updateInterval;
  public: std::chrono::duration<double> nextUpdateTime{0.0};
  public: std::chrono::duration<double> currentTime;
  public: std::chrono::duration<double> lastUpdateTime{0.0};
  public: std::mutex mutex;
  public: gz::common::ConnectionPtr connection{nullptr};
  public: sim::Entity entity = sim::kNullEntity;
  public: sim::EntityComponentManager *ecm{nullptr};

  // Per-instance RNG for observation noise only
  public: std::mt19937 rng;
  public: std::normal_distribution<double> normalDist{0.0, 1.0};

  // Optional seed for reproducible field evolution across launches. If not set, each launch will have a different random field evolution but all buoys within a launch will share the same field evolution.
  public: bool fieldSeedProvided{false};
  public: uint32_t fieldSeed{0};
};

std::map<std::string, gz::msgs::Color>
  FieldLightBuoyPlugin::Implementation::kColors =
  {
    {"red",    CreateColor(1.0, 0.0, 0.0, 1.0)},
    {"green",  CreateColor(0.0, 1.0, 0.0, 1.0)},
    {"blue",   CreateColor(0.0, 0.0, 1.0, 1.0)},
    {"yellow", CreateColor(1.0, 1.0, 0.0, 1.0)},
    {"off",    CreateColor(0.0, 0.0, 0.0, 1.0)},
  };

msgs::Color FieldLightBuoyPlugin::Implementation::CreateColor(
  const double _r, const double _g, const double _b, const double _a)
{
  static msgs::Color color;
  color.set_r(_r);
  color.set_g(_g);
  color.set_b(_b);
  color.set_a(_a);
  return color;
}

double FieldLightBuoyPlugin::Implementation::SharedSampleGaussian(double _mean, double _stddev){
  return _mean + _stddev * s_sharedNormalDist(s_sharedRng);
}

bool FieldLightBuoyPlugin::Implementation::ParseSDF(sdf::ElementPtr _sdf){
  if (!_sdf->HasElement("environment")){
    gzerr << "Missing <environment> element" << std::endl;
    return false;
  }

  this->environment = _sdf->GetElement("environment")->Get<std::string>();

  this->updateInterval = 60.0;
  if (_sdf->HasElement("update_interval")){
    this->updateInterval = _sdf->GetElement("update_interval")->Get<double>();
  }

  // Optional <field_seed> for reproducible OU realisation. Falls back to
  // FIELD_SEED env var so launches can override without editing world SDFs.
  if (_sdf->HasElement("field_seed")){
    this->fieldSeed = _sdf->GetElement("field_seed")->Get<uint32_t>();
    this->fieldSeedProvided = true;
    gzmsg << "FieldLightBuoyPlugin: using SDF field_seed=" << this->fieldSeed << std::endl;
  }
  else if (const char *envSeed = std::getenv("FIELD_SEED")){
    try {
      this->fieldSeed = static_cast<uint32_t>(std::stoul(envSeed));
      this->fieldSeedProvided = true;
      gzmsg << "FieldLightBuoyPlugin: using FIELD_SEED env var=" << this->fieldSeed << std::endl;
    }
    catch (const std::exception &e){
      gzwarn << "FieldLightBuoyPlugin: FIELD_SEED env var '" << envSeed
             << "' is not a valid uint32 (" << e.what() << "); falling back to random_device" << std::endl;
    }
  }

  if (!_sdf->HasElement("visuals")){
    gzerr << "<visuals> missing" << std::endl;
    return false;
  }

  auto visualsElem = _sdf->GetElement("visuals");
  if (!visualsElem->HasElement("visual")){
    gzerr << "<visual> missing" << std::endl;
    return false;
  }

  auto visualElem = visualsElem->GetElement("visual");
  while (visualElem){
    std::string visualName = visualElem->Get<std::string>();
    this->visualNames.push_back(visualName);
    visualElem = visualElem->GetNextElement();
  }

  gzmsg << "FieldLightBuoyPlugin configured for: " << this->environment
        << " (update every " << this->updateInterval << "s)" << std::endl;

  return true;
}

// ============================================================================
void FieldLightBuoyPlugin::Implementation::InitializeField(
  const std::string &_environment)
{
  std::random_device rd;
  // this->rng = std::mt19937(rd());
  if (this->fieldSeedProvided){
    uint32_t instanceSeed = this->fieldSeed ^ static_cast<uint32_t>(this->entity);
    this->rng = std::mt19937(instanceSeed);
  }
  else{
    this->rng = std::mt19937(rd());
  }

  {
    std::lock_guard<std::mutex> lock(s_gpMutex);

    if (!s_gpInitialized || s_gpEnvironment != _environment){
      s_sharedGPs.clear();
      // s_sharedRng = std::mt19937(rd());  // Random seed
      if (this->fieldSeedProvided){
        s_sharedRng = std::mt19937(this->fieldSeed);
        gzmsg << "FieldLightBuoyPlugin: shared GP RNG seeded with field_seed=" << this->fieldSeed << std::endl;
      }
      else{
        s_sharedRng = std::mt19937(rd());
      }

      if (_environment == "uniform_distrib_env"){
        // Left centroid (-496, 231), Right centroid (-368, 231)
        GaussianProcessParams gp1;
        gp1.centerX = -496.0;
        gp1.centerY = 231.0;
        gp1.spatialLengthScale = 80.0;
        gp1.temporalLengthScale = 60.0;
        gp1.variance = 0.4;
        gp1.meanValue = 0.5;
        gp1.noiseStdDev = 0.05;
        gp1.meanReversionRate = 0.000624;
        gp1.diffusionCoeff = 0.15;
        gp1.currentAmplitude = SharedSampleGaussian(gp1.meanValue, std::sqrt(gp1.variance));
        s_sharedGPs.push_back(gp1);

        GaussianProcessParams gp2;
        gp2.centerX = -368.0;
        gp2.centerY = 231.0;
        gp2.spatialLengthScale = 80.0;
        gp2.temporalLengthScale = 80.0;
        gp2.variance = 0.4;
        gp2.meanValue = 0.5;
        gp2.noiseStdDev = 0.05;
        gp2.meanReversionRate = 0.000624;
        gp2.diffusionCoeff = 0.18;
        gp2.currentAmplitude = SharedSampleGaussian(gp2.meanValue, std::sqrt(gp2.variance));
        s_sharedGPs.push_back(gp2);

        gzmsg << "Initialized UNIFORM environment with shared GP fields" << std::endl;
      }

      else if (_environment == "clustered_distrib_env"){
        GaussianProcessParams gp1;
        gp1.centerX = -528.0;
        gp1.centerY = 199.0;
        gp1.spatialLengthScale = 25.0;
        gp1.temporalLengthScale = 50.0;
        gp1.variance = 0.36;
        gp1.meanValue = 0.6;
        gp1.noiseStdDev = 0.08;
        gp1.meanReversionRate = 0.001224;
        gp1.diffusionCoeff = 0.20;
        gp1.currentAmplitude = SharedSampleGaussian(gp1.meanValue, std::sqrt(gp1.variance));
        s_sharedGPs.push_back(gp1);

        GaussianProcessParams gp2;
        gp2.centerX = -448.0;
        gp2.centerY = 246.0;
        gp2.spatialLengthScale = 25.0;
        gp2.temporalLengthScale = 60.0;
        gp2.variance = 0.36;
        gp2.meanValue = 0.6;
        gp2.noiseStdDev = 0.08;
        gp2.meanReversionRate = 0.001224;
        gp2.diffusionCoeff = 0.22;
        gp2.currentAmplitude = SharedSampleGaussian(gp2.meanValue, std::sqrt(gp2.variance));
        s_sharedGPs.push_back(gp2);

        gzmsg << "Initialized CLUSTERED environment with shared GP fields" << std::endl;
      }
      else if (_environment == "boundary_distrib_env"){
        // Single large GP centered on the buoy field.
        GaussianProcessParams gp1;
        gp1.centerX = -448.0;
        gp1.centerY = 241.0; 
        gp1.spatialLengthScale = 95.0;
        gp1.temporalLengthScale = 80.0;
        gp1.variance = 0.25;
        gp1.meanValue = 0.6;
        gp1.noiseStdDev = 0.05;
        gp1.meanReversionRate = 0.000623;
        gp1.diffusionCoeff = 0.12;
        gp1.currentAmplitude = SharedSampleGaussian(gp1.meanValue, std::sqrt(gp1.variance));
        s_sharedGPs.push_back(gp1);

        gzmsg << "Initialized BOUNDARY environment with single large GP" << std::endl;
      }

      if (!s_sharedGPs.empty()){
        for (size_t i = 0; i < s_sharedGPs.size(); ++i){
          gzmsg << "  GP" << (i+1) << " initial amplitude: "
                << s_sharedGPs[i].currentAmplitude << std::endl;
        }
      }

      s_gpEnvironment = _environment;
      s_gpInitialized = true;
      s_lastGPUpdateTimeSec = -1.0;
      s_lastColorPubTimeSec = -1.0;

      // Create Gazebo transport publisher (once)
      if (!s_ownershipDetermined && !s_sharedGPs.empty()){
        std::vector<gz::transport::MessagePublisher> pubs;
        s_transportNode.TopicInfo("/buoy_visual_colors", pubs);

        if (pubs.empty()){
          s_gpPub = s_transportNode.Advertise<gz::msgs::StringMsg>("/gp_field_state");
          s_colorPub = s_transportNode.Advertise<gz::msgs::StringMsg>("/buoy_visual_colors");

          s_publisherCreated = true;
          s_isComputeOwner = true;

          gzmsg << "COMPUTE OWNER" << std::endl;
        } 
        else {
          s_isComputeOwner = false;
          gzmsg << "CONSUMER (GUI)" << std::endl;
        }

        s_ownershipDetermined = true;
      }
    }
  }

  // ===== PER-INSTANCE FIELD TYPE SETUP =====
  if (_environment == "uniform_distrib_env" ||
      _environment == "clustered_distrib_env" ||
      _environment == "boundary_distrib_env")
  {
    this->fieldType = "gaussian_process";
    this->baseValue = 0.0;
  }
  
  else
  {
    gzerr << "Unknown environment: " << _environment << std::endl;
    gzerr << "Defaulting to uniform_distrib_env" << std::endl;
    this->InitializeField("uniform_distrib_env");
  }
}

// ============================================================================
void FieldLightBuoyPlugin::Implementation::UpdateSharedGaussianProcesses(double _dt){
  for (auto &gp : s_sharedGPs){
    double theta = gp.meanReversionRate;
    double mu = gp.meanValue;
    double sigma = gp.diffusionCoeff;

    double decay = std::exp(-theta * _dt);

    // Conditional mean: decays toward mu
    double condMean = mu + (gp.currentAmplitude - mu) * decay;

    // Conditional standard deviation
    double condVar = (sigma * sigma / (2.0 * theta)) * (1.0 - std::exp(-2.0 * theta * _dt));
    double condStd = std::sqrt(std::max(0.0, condVar));

    // Sample new amplitude
    gp.currentAmplitude = condMean + condStd * s_sharedNormalDist(s_sharedRng);

    // Soft clamp to keep field reasonable
    gp.currentAmplitude = std::max(0.0, std::min(1.2, gp.currentAmplitude));
  }
}

// ============================================================================
void FieldLightBuoyPlugin::Implementation::PublishGPState(double _simTime){
  if (!s_publisherCreated || s_sharedGPs.empty())
    return;

  gz::msgs::StringMsg msg;

  std::ostringstream ss;
  ss << std::setprecision(8);
  ss << _simTime << "," << s_sharedGPs.size();

  for (const auto &gp : s_sharedGPs){
    ss << "," << gp.currentAmplitude
       << "," << gp.centerX
       << "," << gp.centerY
       << "," << gp.spatialLengthScale
       << "," << gp.meanValue
       << "," << gp.noiseStdDev;
  }

  ss << "," << s_buoyCache.size();
  for (const auto &pair : s_buoyCache){
    const auto &b = pair.second;
    ss << "," << b.x << "," << b.y << "," << b.fieldValue << "," << b.color;
  }

  msg.set_data(ss.str());
  s_gpPub.Publish(msg);
}

// ============================================================================
void FieldLightBuoyPlugin::Implementation::PublishBuoyColors(){
  if (!s_publisherCreated)
    return;

  gz::msgs::StringMsg msg;
  std::ostringstream ss;

  bool first = true;
  for (const auto &pair : s_buoyCache){
    if (!first) ss << ",";
    ss << pair.first << ":" << pair.second.color;
    first = false;
  }

  msg.set_data(ss.str());
  s_colorPub.Publish(msg);
}

// ============================================================================
double FieldLightBuoyPlugin::Implementation::EvaluateGaussianProcessField(double _x, double _y, double _t){
  // For spatial correlation
  std::lock_guard<std::mutex> lock(s_gpMutex);

  double fieldValue = this->baseValue;

  for (const auto &gp : s_sharedGPs){
    double dx = _x - gp.centerX;
    double dy = _y - gp.centerY;
    double distSq = dx * dx + dy * dy;
    double spatialCorr = std::exp(-distSq / (2.0 * gp.spatialLengthScale * gp.spatialLengthScale));

    // Per-instance observation noise
    double noise = gp.noiseStdDev * this->normalDist(this->rng);
    double contribution = gp.currentAmplitude * spatialCorr + noise;

    fieldValue += contribution;
  }

  fieldValue = std::max(0.0, std::min(1.2, fieldValue));
  return fieldValue;
}

// ============================================================================
double FieldLightBuoyPlugin::Implementation::EvaluateGradientField(double _x, double _y, double _t){
  double amplitude = this->gradientAmplitudeBase +
                    this->gradientAmplitudeRange *
                    std::sin(2.0 * M_PI * _t / this->gradientPeriod);

  double normalizedX = (_x - this->xRangeMin) / (this->xRangeMax - this->xRangeMin);
  normalizedX = std::max(0.0, std::min(1.0, normalizedX));

  double gradientValue = this->minValue +
                        (this->maxValue - this->minValue) * normalizedX;
  gradientValue *= amplitude;

  if (this->noiseLevel > 0.0){
    double noise = this->noiseLevel * std::sin(_x * 0.1) * std::cos(_y * 0.1);
    gradientValue += noise;
  }

  return gradientValue;
}

// ============================================================================
double FieldLightBuoyPlugin::Implementation::EvaluateScalarField(double _x, double _y, double _t){
  if (this->fieldType == "gaussian_process"){
    return this->EvaluateGaussianProcessField(_x, _y, _t);
  }
  
  else if (this->fieldType == "linear_gradient"){
    return this->EvaluateGradientField(_x, _y, _t);
  }

  gzerr << "Unknown field type: " << this->fieldType << std::endl;
  return 0.0;
}

// ============================================================================
std::string FieldLightBuoyPlugin::Implementation::FieldValueToColor(
  double _fieldValue)
{
  for (size_t i = 0; i < this->colorThresholds.size() - 1; ++i){
    if (_fieldValue >= this->colorThresholds[i] && _fieldValue < this->colorThresholds[i + 1]){
      return this->colorNames[i];
    }
  }

  return this->colorNames.back();
}

// ============================================================================
bool FieldLightBuoyPlugin::Implementation::GetBuoyPosition(double &_x, double &_y){
  if (!this->ecm)
    return false;

  auto parentLinkComp = this->ecm->Component<sim::components::ParentEntity>(this->entity);
  if (!parentLinkComp){
    gzerr << "Failed to get parent link for visual entity " << this->entity << std::endl;
    return false;
  }
  sim::Entity linkEntity = parentLinkComp->Data();

  auto parentModelComp = this->ecm->Component<sim::components::ParentEntity>(linkEntity);
  if (!parentModelComp){
    gzerr << "Failed to get parent model for link entity " << linkEntity << std::endl;
    return false;
  }
  sim::Entity modelEntity = parentModelComp->Data();

  auto worldPose = sim::worldPose(modelEntity, *this->ecm);
  _x = worldPose.Pos().X();
  _y = worldPose.Pos().Y();

  return true;
}

// ============================================================================
void FieldLightBuoyPlugin::Implementation::OnBuoyColors(
  const gz::msgs::StringMsg &_msg)
{
  std::lock_guard<std::mutex> lock(s_rxColorMutex);
  s_receivedColors.clear();

  const std::string &data = _msg.data();
  if (data.empty()) return;

  std::istringstream ss(data);
  std::string pair;
  while (std::getline(ss, pair, ','))
  {
    auto colonPos = pair.find(':');
    if (colonPos == std::string::npos) continue;
    try {
      uint64_t eid = std::stoull(pair.substr(0, colonPos));
      s_receivedColors[eid] = pair.substr(colonPos + 1);
    } catch (...) {
      continue;
    }
  }
}

// ============================================================================
void FieldLightBuoyPlugin::Implementation::Update(){
  if (!this->scene)
    this->scene = rendering::sceneFromFirstRenderEngine();

  if (!this->scene)
    return;

  if (this->visuals.empty()){
    auto rootVis = this->scene->RootVisual();
    std::list<rendering::NodePtr> nodes;
    nodes.push_back(rootVis);
    rendering::VisualPtr visual;

    while (!nodes.empty())
    {
      auto n = nodes.front();
      nodes.pop_front();
      if (n && n->HasUserData("gazebo-entity"))
      {
        auto variant = n->UserData("gazebo-entity");
        const uint64_t *value = std::get_if<uint64_t>(&variant);
        if (value && *value == static_cast<uint64_t>(this->entity)){
          visual = std::dynamic_pointer_cast<rendering::Visual>(n);
          break;
        }
      }

      for (unsigned int i = 0u; i < n->ChildCount(); ++i)
        nodes.push_back(n->ChildByIndex(i));
    }

    if (!visual) return;

    rendering::VisualPtr linkVisual =
      std::dynamic_pointer_cast<rendering::Visual>(visual->Parent());

    if (!linkVisual) return;

    for (const auto &name : this->visualNames){
      rendering::NodePtr node = linkVisual->ChildByName(name);

      if (!node){
        auto delim = name.rfind("/");
        auto shortName = name.substr(delim + 1);
        node = linkVisual->ChildByName(shortName);
      }

      if (node){
        auto v = std::dynamic_pointer_cast<rendering::Visual>(node);
        if (v)
          this->visuals.push_back(v);
      }
      
      else{
        gzerr << "Unable to find visual: " << name << std::endl;
      }
    }
  }

  if (!s_isComputeOwner)
  {
    if (!s_subscribedToColors){
      s_transportNode.Subscribe("/buoy_visual_colors", &OnBuoyColors);
      s_subscribedToColors = true;
    }

    std::string colorName = "off";
    {
      std::lock_guard<std::mutex> rxLock(s_rxColorMutex);
      auto it = s_receivedColors.find(static_cast<uint64_t>(this->entity));
      if (it != s_receivedColors.end())
        colorName = it->second;
    }

    auto color = this->kColors[colorName];
    for (auto vis : this->visuals){
      math::Color gc(color.r(), color.g(), color.b(), color.a());
      auto mat = vis->Material();
      if (!mat){
        auto newMat = this->scene->CreateMaterial();
        vis->SetMaterial(newMat);
        mat = vis->Material();
        this->scene->DestroyMaterial(newMat);
      }
      mat->SetAmbient(gc);
      mat->SetDiffuse(gc);
    }
    return;
  }

  std::lock_guard<std::mutex> lock(this->mutex);

  // Heartbeat: republish cached colors at ~5 Hz so late-attaching
  // subscribers (GUI, Python viz) catch up within a frame instead of
  // waiting a full updateInterval (480 s) for the next GP cycle.
  {
    std::lock_guard<std::mutex> gpLock(s_gpMutex);
    double nowSec = this->currentTime.count();
    if (!s_buoyCache.empty() &&
        (s_lastColorPubTimeSec < 0.0 || nowSec - s_lastColorPubTimeSec >= 0.2)){
      PublishBuoyColors();
      PublishGPState(nowSec);
      s_lastColorPubTimeSec = nowSec;
    }
  }

  if (this->currentTime < this->nextUpdateTime)
    return;

  double simTimeSec = this->currentTime.count();


  if (this->fieldType == "gaussian_process"){
    std::lock_guard<std::mutex> gpLock(s_gpMutex);

    if (simTimeSec > s_lastGPUpdateTimeSec){
      double dt = (s_lastGPUpdateTimeSec < 0.0)
                  ? this->updateInterval
                  : (simTimeSec - s_lastGPUpdateTimeSec);

      if (dt > 0.0){
        UpdateSharedGaussianProcesses(dt);
        // PublishGPState is deferred until all buoys evaluate

        gzmsg << "GP Update t=" << simTimeSec << "s:";
        for (size_t i = 0; i < s_sharedGPs.size(); ++i)
        {
          gzmsg << " GP" << (i+1) << "=" << s_sharedGPs[i].currentAmplitude;
        }
        gzmsg << std::endl;
      }

      s_lastGPUpdateTimeSec = simTimeSec;
    }
  }

  this->nextUpdateTime += std::chrono::duration<double>(this->updateInterval);
  this->lastUpdateTime = this->currentTime;

  double buoyX, buoyY;
  if (!this->GetBuoyPosition(buoyX, buoyY)){
    gzerr << "Failed to get buoy position" << std::endl;
    return;
  }

  double fieldValue = this->EvaluateScalarField(buoyX, buoyY, simTimeSec);
  std::string colorName = this->FieldValueToColor(fieldValue);

  // Cache this buoy's evaluated result and publish when all buoys are done
  if (this->fieldType == "gaussian_process")
  {
    std::lock_guard<std::mutex> gpLock(s_gpMutex);
    uint64_t eid = static_cast<uint64_t>(this->entity);
    s_buoyCache[eid] = {eid, buoyX, buoyY, fieldValue, colorName};

    s_buoysEvaluatedThisCycle++;
    if (s_buoysEvaluatedThisCycle >= s_totalBuoyInstances && s_totalBuoyInstances > 0){
      PublishGPState(simTimeSec);
      PublishBuoyColors();
      s_buoysEvaluatedThisCycle = 0;
    }
  }

  gzmsg << "Buoy at (" << buoyX << ", " << buoyY << ") | "
        << "t=" << simTimeSec << "s | "
        << "Field=" << fieldValue << " -> " << colorName << std::endl;

  auto color = this->kColors[colorName];

  for (auto visual : this->visuals){
    math::Color gazeboColor(color.r(), color.g(), color.b(), color.a());

    auto mat = visual->Material();
    if (!mat){
      auto newMat = this->scene->CreateMaterial();
      visual->SetMaterial(newMat);
      mat = visual->Material();
      this->scene->DestroyMaterial(newMat);
    }
    mat->SetAmbient(gazeboColor);
    mat->SetDiffuse(gazeboColor);
  }
}

// ============================================================================
FieldLightBuoyPlugin::FieldLightBuoyPlugin() : dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

// ============================================================================
void FieldLightBuoyPlugin::Configure(
  const sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  sim::EntityComponentManager &_ecm,
  sim::EventManager &_eventMgr)
{
  this->dataPtr->entity = _entity;
  this->dataPtr->ecm = &_ecm;

  auto sdf = _sdf->Clone();
  if (!this->dataPtr->ParseSDF(sdf)){
    gzerr << "Error parsing SDF, plugin disabled." << std::endl;
    return;
  }

  auto worldEntity = _ecm.EntityByComponents(sim::components::World());
  auto worldNameComp = _ecm.Component<sim::components::Name>(worldEntity);
  if (worldNameComp){
    std::string worldName = worldNameComp->Data();
    // Strip weather suffix so clustered/boundary/uniform_distrib_env_{fog,night}
    // all map to the same GP parameters as the base env.
    for (const std::string &suffix : {"_fog", "_night"}){
      if (worldName.size() > suffix.size() &&
          worldName.compare(worldName.size() - suffix.size(),
                            suffix.size(), suffix) == 0){
        worldName.erase(worldName.size() - suffix.size());
        break;
      }
    }
    this->dataPtr->environment = worldName;
    gzmsg << "FieldLightBuoyPlugin: auto-detected environment from world name: "
          << worldNameComp->Data() << " -> " << this->dataPtr->environment << std::endl;
  }
  
  else{
    gzwarn << "FieldLightBuoyPlugin: could not read world name, "
           << "falling back to SDF environment: "
           << this->dataPtr->environment << std::endl;
  }

  this->dataPtr->InitializeField(this->dataPtr->environment);

  if (this->dataPtr->fieldType == "gaussian_process" && s_isComputeOwner){
    std::lock_guard<std::mutex> lock(s_gpMutex);
    s_totalBuoyInstances++;
    gzmsg << "FieldLightBuoy instance #" << s_totalBuoyInstances
          << " registered" << std::endl;
  }

  this->dataPtr->connection =
    _eventMgr.Connect<sim::events::SceneUpdate>(
      std::bind(&FieldLightBuoyPlugin::Implementation::Update,
                this->dataPtr.get()));
}

// ============================================================================
void FieldLightBuoyPlugin::PreUpdate(const sim::UpdateInfo &_info, sim::EntityComponentManager &_ecm){
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->currentTime = _info.simTime;
}

GZ_ADD_PLUGIN(FieldLightBuoyPlugin,
              sim::System,
              FieldLightBuoyPlugin::ISystemConfigure,
              FieldLightBuoyPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(vrx::FieldLightBuoyPlugin,
                    "vrx::FieldLightBuoyPlugin")