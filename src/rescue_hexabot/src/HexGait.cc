// HexGait: alternating-tripod gait for an RHex-style robot with six rotating C-legs.
//
// Subscribes to an ignition.msgs.Twist and drives each hip joint with a velocity
// command that tracks a "Buehler clock": the leg sweeps slowly through the stance
// arc around "pointing down" (theta = 0) and quickly over the top during swing.
// Each side has its own phase so a speed difference between the sides turns the
// robot; on each side the front and rear legs are half a cycle away from the
// middle leg, so at least one leg per side is always in stance.
//
// SDF parameters:
//   <topic>               twist input
//   <leg><joint/><side>left|right</side><offset>0|0.5</offset></leg>   x6
//   <stride>              metres travelled per gait cycle (calibrated)
//   <track_width>         effective distance between the left and right leg rows
//   <stance_sweep>        stance arc [rad]
//   <duty>                fraction of the cycle in stance
//   <max_frequency>       gait cycles per second limit
//   <frequency_accel>     cycles/s^2 ramp limit
//   <kp>                  position-error gain [1/s]
//   <max_joint_velocity>  [rad/s]
//   <cmd_timeout>         [s] stop and stand if no command arrives
//   <climb_tilt_start>    [rad] tilt where the gait starts slowing down
//   <climb_tilt_full>     [rad] tilt where it reaches <climb_min_scale>
//   <climb_min_scale>     slowest gait while climbing, as a fraction of the command
//   <flip_guard_pitch>    [rad] nose-up pitch where the legs stop driving so the
//                         body settles back instead of flipping over backwards

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/msgs/twist.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace rescue_hexabot
{
using namespace ignition;
using namespace ignition::gazebo;

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;

double wrapPi(double a)
{
  a = std::fmod(a + M_PI, kTwoPi);
  return a < 0 ? a + M_PI : a - M_PI;
}

double wrap01(double p)
{
  p = std::fmod(p, 1.0);
  return p < 0 ? p + 1.0 : p;
}
}  // namespace

class HexGait : public System, public ISystemConfigure, public ISystemPreUpdate
{
public:
  void Configure(const Entity & entity, const std::shared_ptr<const sdf::Element> & sdfIn,
                 EntityComponentManager & ecm, EventManager &) override
  {
    Model model(entity);
    model_ = entity;
    auto sdf = sdfIn->Clone();
    auto get = [&](const char * name, double def) {
      return sdf->HasElement(name) ? sdf->Get<double>(name) : def;
    };
    stride_ = get("stride", 0.25);
    track_ = get("track_width", 0.35);
    sweep_ = get("stance_sweep", M_PI / 2);
    duty_ = get("duty", 0.5);
    maxFreq_ = get("max_frequency", 3.0);
    freqAccel_ = get("frequency_accel", 6.0);
    kp_ = get("kp", 25.0);
    maxJointVel_ = get("max_joint_velocity", 35.0);
    timeout_ = get("cmd_timeout", 0.5);
    tiltStart_ = get("climb_tilt_start", 0.2);
    tiltFull_ = get("climb_tilt_full", 0.5);
    minScale_ = get("climb_min_scale", 0.35);
    flipPitch_ = get("flip_guard_pitch", 0.7);

    for (auto e = sdf->GetElement("leg"); e; e = e->GetNextElement("leg")) {
      Leg leg;
      const auto name = e->Get<std::string>("joint");
      leg.joint = model.JointByName(ecm, name);
      leg.left = e->Get<std::string>("side") == "left";
      leg.offset = e->Get<double>("offset");
      if (leg.joint == kNullEntity) {
        ignerr << "[HexGait] joint not found: " << name << "\n";
        continue;
      }
      if (!ecm.Component<components::JointPosition>(leg.joint)) {
        ecm.CreateComponent(leg.joint, components::JointPosition());
      }
      legs_.push_back(leg);
    }

    const auto topic = sdf->Get<std::string>("topic");
    node_.Subscribe(topic, &HexGait::OnTwist, this);
    ignmsg << "[HexGait] " << legs_.size() << " legs, listening on " << topic << "\n";
  }

  void PreUpdate(const UpdateInfo & info, EntityComponentManager & ecm) override
  {
    if (info.paused || legs_.empty()) {
      return;
    }
    const double dt = std::chrono::duration<double>(info.dt).count();
    const double now = std::chrono::duration<double>(info.simTime).count();

    double v = 0.0, w = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (newCmd_) {
        lastCmdTime_ = now;
        newCmd_ = false;
      }
      if (now - lastCmdTime_ < timeout_) {
        v = cmdV_;
        w = cmdW_;
      }
    }

    // Climbing: slow down with body tilt; stop driving when about to flip backwards.
    const auto pose = worldPose(model_, ecm);
    const double tilt = std::acos(std::clamp(pose.Rot().RotateVector({0, 0, 1}).Z(), -1.0, 1.0));
    const double noseUp = std::asin(std::clamp(pose.Rot().RotateVector({1, 0, 0}).Z(), -1.0, 1.0));
    if (noseUp > flipPitch_) {
      flipGuard_ = true;
    } else if (noseUp < flipPitch_ * 0.5) {
      flipGuard_ = false;
    }
    const double climb = std::clamp((tilt - tiltStart_) / (tiltFull_ - tiltStart_), 0.0, 1.0);
    const double scale = flipGuard_ ? 0.0 : 1.0 - climb * (1.0 - minScale_);

    // Side speeds -> gait frequencies, scaled together so turning keeps its ratio.
    double fL = scale * (v - w * track_ / 2.0) / stride_;
    double fR = scale * (v + w * track_ / 2.0) / stride_;
    const double peak = std::max(std::abs(fL), std::abs(fR));
    if (peak > maxFreq_) {
      fL *= maxFreq_ / peak;
      fR *= maxFreq_ / peak;
    }
    const double step = freqAccel_ * dt;
    freqL_ += std::clamp(fL - freqL_, -step, step);
    freqR_ += std::clamp(fR - freqR_, -step, step);

    const bool moving = std::abs(freqL_) > 0.02 || std::abs(freqR_) > 0.02;
    if (moving) {
      phaseL_ = wrap01(phaseL_ + freqL_ * dt);
      phaseR_ = wrap01(phaseR_ + freqR_ * dt);
    } else {
      // Restart every gait from the same coordinated state.
      phaseL_ = phaseR_ = 0.0;
    }

    for (const auto & leg : legs_) {
      const auto * pos = ecm.Component<components::JointPosition>(leg.joint);
      if (!pos || pos->Data().empty()) {
        continue;
      }
      double target = 0.0, ff = 0.0;
      if (moving) {
        const double f = leg.left ? freqL_ : freqR_;
        Clock(wrap01((leg.left ? phaseL_ : phaseR_) + leg.offset), f, target, ff);
      }
      // When idle every leg returns to "pointing down" and the robot stands on all six.
      const double err = wrapPi(target - pos->Data()[0]);
      const double cmd = std::clamp(ff + kp_ * err, -maxJointVel_, maxJointVel_);
      auto * vel = ecm.Component<components::JointVelocityCmd>(leg.joint);
      if (vel) {
        vel->Data() = {cmd};
      } else {
        ecm.CreateComponent(leg.joint, components::JointVelocityCmd({cmd}));
      }
    }
  }

private:
  struct Leg
  {
    Entity joint{kNullEntity};
    bool left{true};
    double offset{0.0};
  };

  // Buehler clock: phase in [0,1) -> leg angle and its rate for gait frequency f.
  void Clock(double p, double f, double & theta, double & rate) const
  {
    if (p < duty_) {
      theta = -sweep_ / 2.0 + sweep_ * p / duty_;
      rate = sweep_ / duty_ * f;
    } else {
      theta = sweep_ / 2.0 + (kTwoPi - sweep_) * (p - duty_) / (1.0 - duty_);
      rate = (kTwoPi - sweep_) / (1.0 - duty_) * f;
    }
  }

  void OnTwist(const msgs::Twist & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cmdV_ = msg.linear().x();
    cmdW_ = msg.angular().z();
    newCmd_ = true;
  }

  transport::Node node_;
  Entity model_{kNullEntity};
  std::vector<Leg> legs_;
  bool flipGuard_{false};
  std::mutex mutex_;
  double cmdV_{0.0}, cmdW_{0.0};
  bool newCmd_{false};
  double lastCmdTime_{-1e9};

  double stride_, track_, sweep_, duty_, maxFreq_, freqAccel_, kp_, maxJointVel_, timeout_;
  double tiltStart_, tiltFull_, minScale_, flipPitch_;
  double freqL_{0.0}, freqR_{0.0}, phaseL_{0.0}, phaseR_{0.0};
};
}  // namespace rescue_hexabot

IGNITION_ADD_PLUGIN(rescue_hexabot::HexGait, ignition::gazebo::System,
                    rescue_hexabot::HexGait::ISystemConfigure,
                    rescue_hexabot::HexGait::ISystemPreUpdate)
