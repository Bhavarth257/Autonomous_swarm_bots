// Drive, suspension and levelling for the wheel-legged rocker robot (wheelleg.urdf.xacro).
//
// Joints (by name): l/r_rocker_joint (free hub), lf/lr/rf/rr_leg_joint (one motor per side
// opens that side's V: front and rear leg always get the same angle), lf/lr/rf/rr_wheel_joint.
//
// Every physics step:
//  * differential: the left rocker turns freely; the right one is driven to mirror it
//    (velocity -v_l, plus a correction towards q_r = -q_l), which is what a differential
//    bar enforces: when one rocker tips up the other tips down and the body sits at their
//    average pitch. Turning on the spot, the wheels' traction twists the two rockers
//    opposite ways (the high hub turns it into a wind-up that can roll the robot over), so
//    while turning their range is narrowed from rocker_free to rocker_turn (a suspension
//    lock, done with joint limits: any force on the rockers stops the motors responding);
//  * stance and levelling: each side's V is driven to stance +/- a levelling offset that
//    integrates the body's roll (one side lower -> open the other side's V a bit more);
//  * wheels: skid steering from cmd_vel, plus whatever spin keeps each tyre rolling while
//    the plugin itself opens or closes a V (a front wheel moving forward as the V opens
//    must turn forward); without that the tyres pin the feet and the legs can't move.
//    It uses the stance rate the plugin commands, not the measured leg motion: feeding
//    that back made wheels and legs drive each other into an oscillation.
//
// Every motor is velocity-commanded (servo). Gazebo Fortress stops applying velocity
// commands on the legs as soon as any force or torque acts on the rockers (joint force
// commands and link wrenches alike), so the differential can't be a spring-damper torque;
// torque-controlled wheels also chattered against the ground contact.
//
//   /cmd_vel_topic  ignition.msgs.Twist   linear.x, angular.z
//   /stance_topic   ignition.msgs.Double  V half-angle, rad (0 = legs straight down)

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointPositionLimitsCmd.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/msgs/double.pb.h>
#include <ignition/msgs/twist.pb.h>
#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace rescue_hexabot
{
using namespace ignition;
using namespace ignition::gazebo;

namespace
{
double ramp(double cur, double target, double step)
{
  return cur + std::clamp(target - cur, -step, step);
}

void setCmd(EntityComponentManager & ecm, Entity j, double v)
{
  auto * c = ecm.Component<components::JointVelocityCmd>(j);
  if (c) {
    c->Data() = {v};
  } else {
    ecm.CreateComponent(j, components::JointVelocityCmd({v}));
  }
}

double first(const EntityComponentManager & ecm, Entity j, bool velocity)
{
  if (velocity) {
    const auto * c = ecm.Component<components::JointVelocity>(j);
    return (c && !c->Data().empty()) ? c->Data()[0] : 0.0;
  }
  const auto * c = ecm.Component<components::JointPosition>(j);
  return (c && !c->Data().empty()) ? c->Data()[0] : 0.0;
}
}  // namespace

class WheelLegDrive : public System, public ISystemConfigure, public ISystemPreUpdate
{
public:
  void Configure(const Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
                 EntityComponentManager &, EventManager &) override
  {
    model_ = Model(entity);
    auto get = [&](const char * k, double d) { return sdf->Get<double>(k, d).first; };
    bodyName_ = sdf->Get<std::string>("body_link", "base_link").first;
    legL_ = get("leg_length", 0.36);
    wheelR_ = get("wheel_radius", 0.14);
    track_ = get("track", 0.48);
    stance_ = get("stance", 0.6);
    stanceMin_ = get("stance_min", 0.25);
    stanceMax_ = get("stance_max", 1.15);
    legRate_ = get("leg_max_rate", 1.2);
    legGain_ = get("leg_gain", 6.0);
    diffGain_ = get("diff_gain", 8.0);
    diffFilter_ = get("diff_filter", 0.05);
    rockerFree_ = get("rocker_free", 0.45);
    rockerTurn_ = get("rocker_turn", 0.12);
    rockerLimit_ = rockerFree_;
    levelGain_ = get("level_gain", 1.5);
    levelLimit_ = get("level_limit", 0.3);
    maxSpeed_ = get("max_speed", 0.9);
    maxTurn_ = get("max_turn", 2.0);
    accel_ = get("accel", 1.2);
    turnAccel_ = get("turn_accel", 4.0);
    timeout_ = get("cmd_timeout", 0.5);
    debug_ = sdf->Get<bool>("debug", false).first;
    debugPeriod_ = get("debug_period", 1.0);
    stanceCmd_ = stance_;
    const auto cmdTopic = sdf->Get<std::string>("cmd_vel_topic", "/cmd_vel").first;
    const auto stanceTopic = sdf->Get<std::string>("stance_topic", "/stance").first;
    node_.Subscribe(cmdTopic, &WheelLegDrive::OnCmd, this);
    node_.Subscribe(stanceTopic, &WheelLegDrive::OnStance, this);
  }

  void PreUpdate(const UpdateInfo & info, EntityComponentManager & ecm) override
  {
    if (info.paused) {
      return;
    }
    if (!ready_) {
      Init(ecm);
      return;
    }
    const double t = std::chrono::duration<double>(info.simTime).count();
    const double dt = std::chrono::duration<double>(info.dt).count();
    double cvx, cwz, stanceTarget;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (pendingCmdStamp_) {           // stamp commands with simulation time
        cmdTime_ = t;
        pendingCmdStamp_ = false;
      }
      if (cmdTime_ > t) {
        cmdTime_ = -1e9;                // simulation was reset
      }
      const bool fresh = t - cmdTime_ < timeout_;
      cvx = fresh ? cmdVx_ : 0.0;
      cwz = fresh ? cmdWz_ : 0.0;
      stanceTarget = stanceCmd_;
    }
    vx_ = ramp(vx_, std::clamp(cvx, -maxSpeed_, maxSpeed_), accel_ * dt);
    wz_ = ramp(wz_, std::clamp(cwz, -maxTurn_, maxTurn_), turnAccel_ * dt);
    stance_ = ramp(stance_, std::clamp(stanceTarget, stanceMin_, stanceMax_), 0.5 * dt);

    // Levelling: integrate the body's roll into a left/right difference of the V angles.
    // Positive roll = left side up: open the left V (lower it), close the right one.
    const double roll = worldPose(body_, ecm).Rot().Roll();
    level_ = std::clamp(level_ + levelGain_ * roll * dt, -levelLimit_, levelLimit_);

    // Differential between the rockers.
    const double qL = first(ecm, rocker_[0], false), qR = first(ecm, rocker_[1], false);
    const double vL = first(ecm, rocker_[0], true), vR = first(ecm, rocker_[1], true);
    (void)vR;
    // The free rocker is light and its measured speed jittery: copy a smoothed version.
    vLf_ += (vL - vLf_) * std::min(1.0, dt / diffFilter_);
    setCmd(ecm, rocker_[1], -vLf_ - diffGain_ * (qL + qR));
    const double wantLimit = std::abs(wz_) > 0.3 ? rockerTurn_ : rockerFree_;
    const double floor = std::max(std::abs(qL), std::abs(qR)) + 0.01;   // never snap a rocker
    rockerLimit_ = std::max(ramp(rockerLimit_, wantLimit, (wantLimit < rockerLimit_ ? 1.0 : 2.0) * dt),
                            std::min(floor, rockerFree_));
    for (Entity r : rocker_) {
      const std::vector<math::Vector2d> lim{math::Vector2d(-rockerLimit_, rockerLimit_)};
      auto * c = ecm.Component<components::JointPositionLimitsCmd>(r);
      if (c) {
        c->Data() = lim;
      } else {
        ecm.CreateComponent(r, components::JointPositionLimitsCmd(lim));
      }
    }

    if (debug_ && t - lastDebug_ > debugPeriod_) {
      lastDebug_ = t;
      ignmsg << "[WheelLegDrive] t=" << t << " x=" << worldPose(body_, ecm).Pos().X()
             << " pitch=" << worldPose(body_, ecm).Rot().Pitch() << " stance=" << stance_ << " level=" << level_
             << " roll=" << roll << " rockers=" << qL << "," << qR
             << " legs=" << first(ecm, leg_[0][0], false) << "," << first(ecm, leg_[0][1], false) << ","
             << first(ecm, leg_[1][0], false) << "," << first(ecm, leg_[1][1], false)
             << " vx=" << vx_ << " wheels(lf,lr)=" << first(ecm, wheel_[0][0], true) << ","
             << first(ecm, wheel_[0][1], true) << std::endl;
    }
    for (int s = 0; s < 2; ++s) {                       // 0 left, 1 right
      const double desired = std::clamp(stance_ + (s == 0 ? level_ : -level_), stanceMin_, stanceMax_);
      const double before = legTarget_[s];
      legTarget_[s] = ramp(legTarget_[s], desired, legRate_ * dt);
      const double target = legTarget_[s];
      const double rate = dt > 0.0 ? (target - before) / dt : 0.0;   // commanded V opening rate
      const double side = s == 0 ? -1.0 : 1.0;           // turning left: left side slower
      const double vSide = vx_ + side * wz_ * track_ / 2.0;
      for (int e = 0; e < 2; ++e) {                      // 0 front, 1 rear
        const double dir = e == 0 ? 1.0 : -1.0;
        const Entity leg = leg_[s][e], wheel = wheel_[s][e];
        const double q = first(ecm, leg, false);
        setCmd(ecm, leg, std::clamp(rate + legGain_ * (target - q), -2.0 * legRate_, 2.0 * legRate_));
        // Leg spin about +y relative to the body (front joint axis is -y, rear +y), and the
        // wheel centre's forward speed relative to the body, from the commanded opening rate.
        const double legSpin = -dir * rate;
        const double xdot = dir * legL_ * std::cos(q) * rate;
        setCmd(ecm, wheel, (vSide + xdot) / wheelR_ - legSpin);
      }
    }
  }

private:
  void Init(EntityComponentManager & ecm)
  {
    body_ = model_.LinkByName(ecm, bodyName_);
    const char * sides = "lr";
    const char * ends = "fr";
    bool ok = body_ != kNullEntity;
    for (int s = 0; s < 2 && ok; ++s) {
      rocker_[s] = model_.JointByName(ecm, std::string(1, sides[s]) + "_rocker_joint");
      ok = ok && rocker_[s] != kNullEntity;
      for (int e = 0; e < 2; ++e) {
        const std::string n = std::string(1, sides[s]) + ends[e];
        leg_[s][e] = model_.JointByName(ecm, n + "_leg_joint");
        wheel_[s][e] = model_.JointByName(ecm, n + "_wheel_joint");
        ok = ok && leg_[s][e] != kNullEntity && wheel_[s][e] != kNullEntity;
      }
    }
    if (!ok) {
      return;
    }
    auto track = [&](Entity j) {
      if (!ecm.Component<components::JointPosition>(j)) {
        ecm.CreateComponent(j, components::JointPosition());
      }
      if (!ecm.Component<components::JointVelocity>(j)) {
        ecm.CreateComponent(j, components::JointVelocity());
      }
    };
    for (int s = 0; s < 2; ++s) {
      track(rocker_[s]);
      for (int e = 0; e < 2; ++e) {
        track(leg_[s][e]);
        track(wheel_[s][e]);
        // Start already standing in the stance (the URDF zero is legs straight down).
        ecm.CreateComponent(leg_[s][e], components::JointPositionReset({stance_}));
      }
    }
    legTarget_ = {stance_, stance_};
    ready_ = true;
  }

  void OnCmd(const msgs::Twist & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cmdVx_ = msg.linear().x();
    cmdWz_ = msg.angular().z();
    pendingCmdStamp_ = true;
  }

  void OnStance(const msgs::Double & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stanceCmd_ = msg.data();
  }

  Model model_{kNullEntity};
  Entity body_{kNullEntity};
  std::array<Entity, 2> rocker_{kNullEntity, kNullEntity};
  std::array<std::array<Entity, 2>, 2> leg_{}, wheel_{};
  bool ready_{false};
  std::string bodyName_;
  double legL_{0.36}, wheelR_{0.14}, track_{0.48};
  double stance_{0.6}, stanceMin_{0.25}, stanceMax_{1.15}, legRate_{1.2};
  double legGain_{6.0};
  std::array<double, 2> legTarget_{0.6, 0.6};
  double diffGain_{8.0}, diffFilter_{0.05}, vLf_{0.0}, levelGain_{1.5}, levelLimit_{0.3}, level_{0.0};
  double rockerFree_{0.45}, rockerTurn_{0.12}, rockerLimit_{0.45};
  double maxSpeed_{0.9}, maxTurn_{2.0}, accel_{1.2}, turnAccel_{4.0}, timeout_{0.5};
  double vx_{0.0}, wz_{0.0};
  transport::Node node_;
  std::mutex mutex_;
  double cmdVx_{0.0}, cmdWz_{0.0}, stanceCmd_{0.6}, cmdTime_{-1e9};
  bool pendingCmdStamp_{false};
  bool debug_{false};
  double debugPeriod_{1.0};
  double lastDebug_{-1e9};
};
}  // namespace rescue_hexabot

IGNITION_ADD_PLUGIN(rescue_hexabot::WheelLegDrive, ignition::gazebo::System,
                    rescue_hexabot::WheelLegDrive::ISystemConfigure,
                    rescue_hexabot::WheelLegDrive::ISystemPreUpdate)
