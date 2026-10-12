// HexWalk: tripod walking gait with inverse kinematics for an 18-DOF spider hexapod.
//
// Each leg has a coxa (yaw), femur (pitch) and tibia (pitch) joint; leg frames have x
// pointing outward from the body. The controller keeps a foot target per leg in the
// body frame:
//   stance  the foot stays planted: it moves backwards at the body's velocity at that
//           point (v + w x r), so the robot moves forwards, sideways or turns;
//   swing   the foot lifts straight up, moves forward at full step height, then drops
//           straight down ahead of its neutral position by half the next stance stroke,
//           so it clears the edges of obstacles instead of catching on them.
// Legs alternate in two tripods (offset half a cycle); with duty > 0.5 both tripods are
// briefly on the ground at each switch. Step frequency rises with speed so the stroke
// stays below <max_stride>. Foot targets go through analytic IK to joint angles, and
// every joint gets a velocity command (feed-forward + P on angle error), so the
// physics engine does the actual walking and the joints have the compliance of a
// torque-limited servo.
//
// When idle, the robot finishes the steps in progress and stands on all six feet.
// The IMU levels the body: legs on the high side shorten and legs on the low side
// extend (integral action on roll/pitch), so feet resting on debris or a ramp don't
// tip the body over. While climbing (body tilt) the gait also slows down.
//
// SDF:
//   <topic>  ignition.msgs.Twist (linear.x, linear.y, angular.z)
//   <leg><name/><mount_x/><mount_y/><mount_yaw/><group>0|1</group></leg> x6
//     joints are <name>_coxa, <name>_femur, <name>_tibia
//   <coxa_length/> <femur_length/> <tibia_length/>
//   <stand_height/>   body origin above the feet
//   <stand_reach/>    neutral horizontal distance from the femur joint to the foot
//   <step_height/> <max_stride/> <duty/> <min_frequency/> <max_frequency/>
//   <max_speed/> <max_yaw_rate/> <accel/> <yaw_accel/>
//   <kp/> <max_joint_velocity/> <cmd_timeout/>
//   <climb_tilt_start/> <climb_tilt_full/> <climb_min_scale/>   (rad, rad, fraction)
//   <joint_state_topic/> <joint_state_rate/>   joint positions as ignition.msgs.Model.
//     Replaces Gazebo's JointStatePublisher, which publishes every physics step
//     (250 Hz) and floods /tf through robot_state_publisher.
//   <level_gain/>     body levelling integral gain [1/s]
//   <level_limit/>    max leg length change from levelling [m]

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
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/msgs/model.pb.h>
#include <ignition/msgs/twist.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace rescue_hexabot
{
using namespace ignition;
using namespace ignition::gazebo;

namespace
{
double wrap01(double p)
{
  p = std::fmod(p, 1.0);
  return p < 0 ? p + 1.0 : p;
}

double smooth(double s)  // smoothstep: zero velocity at lift-off and touch-down
{
  s = std::clamp(s, 0.0, 1.0);
  return s * s * (3.0 - 2.0 * s);
}

double ramp(double cur, double target, double step)
{
  return cur + std::clamp(target - cur, -step, step);
}
}  // namespace

class HexWalk : public System, public ISystemConfigure, public ISystemPreUpdate
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
    l1_ = get("coxa_length", 0.05);
    l2_ = get("femur_length", 0.11);
    l3_ = get("tibia_length", 0.17);
    height_ = get("stand_height", 0.15);
    reach_ = get("stand_reach", 0.14);
    stepHeight_ = get("step_height", 0.09);
    maxStride_ = get("max_stride", 0.12);
    duty_ = get("duty", 0.55);
    minFreq_ = get("min_frequency", 1.0);
    maxFreq_ = get("max_frequency", 3.0);
    maxSpeed_ = get("max_speed", 0.45);
    maxYawRate_ = get("max_yaw_rate", 1.2);
    accel_ = get("accel", 1.0);
    yawAccel_ = get("yaw_accel", 3.0);
    kp_ = get("kp", 30.0);
    maxJointVel_ = get("max_joint_velocity", 12.0);
    timeout_ = get("cmd_timeout", 0.5);
    tiltStart_ = get("climb_tilt_start", 0.2);
    tiltFull_ = get("climb_tilt_full", 0.5);
    minScale_ = get("climb_min_scale", 0.4);
    levelGain_ = get("level_gain", 4.0);
    levelLimit_ = get("level_limit", 0.06);

    for (auto e = sdf->GetElement("leg"); e; e = e->GetNextElement("leg")) {
      Leg leg;
      const auto name = e->Get<std::string>("name");
      leg.mx = e->Get<double>("mount_x");
      leg.my = e->Get<double>("mount_y");
      leg.myaw = e->Get<double>("mount_yaw");
      leg.group = e->Get<int>("group");
      const char * suffix[3] = {"_coxa", "_femur", "_tibia"};
      bool ok = true;
      for (int k = 0; k < 3; ++k) {
        leg.joint[k] = model.JointByName(ecm, name + suffix[k]);
        leg.names[k] = name + suffix[k];
        if (leg.joint[k] == kNullEntity) {
          ignerr << "[HexWalk] joint not found: " << name << suffix[k] << "\n";
          ok = false;
        } else if (!ecm.Component<components::JointPosition>(leg.joint[k])) {
          ecm.CreateComponent(leg.joint[k], components::JointPosition());
        }
      }
      if (!ok) {
        continue;
      }
      // Neutral foot: straight out along the mount direction, on the ground.
      const double r = l1_ + reach_;
      leg.nx = leg.mx + r * std::cos(leg.myaw);
      leg.ny = leg.my + r * std::sin(leg.myaw);
      leg.fx = leg.nx;
      leg.fy = leg.ny;
      leg.lx = leg.nx;
      leg.ly = leg.ny;
      legs_.push_back(leg);
    }

    const auto topic = sdf->Get<std::string>("topic");
    node_.Subscribe(topic, &HexWalk::OnTwist, this);
    if (sdf->HasElement("joint_state_topic")) {
      jsPub_ = node_.Advertise<msgs::Model>(sdf->Get<std::string>("joint_state_topic"));
      jsPeriod_ = 1.0 / get("joint_state_rate", 30.0);
    }
    ignmsg << "[HexWalk] " << legs_.size() << " legs, listening on " << topic << "\n";
  }

  void PreUpdate(const UpdateInfo & info, EntityComponentManager & ecm) override
  {
    if (info.paused || legs_.empty()) {
      return;
    }
    const double dt = std::chrono::duration<double>(info.dt).count();
    const double now = std::chrono::duration<double>(info.simTime).count();
    if (dt <= 0.0) {
      return;
    }

    double cvx = 0.0, cvy = 0.0, cwz = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (newCmd_) {
        lastCmdTime_ = now;
        newCmd_ = false;
      }
      if (now - lastCmdTime_ < timeout_) {
        cvx = cmdVx_;
        cvy = cmdVy_;
        cwz = cmdWz_;
      }
    }

    // Limit and slow down while climbing.
    const double speed = std::hypot(cvx, cvy);
    if (speed > maxSpeed_) {
      cvx *= maxSpeed_ / speed;
      cvy *= maxSpeed_ / speed;
    }
    cwz = std::clamp(cwz, -maxYawRate_, maxYawRate_);
    const auto pose = worldPose(model_, ecm);
    const double tilt = std::acos(std::clamp(pose.Rot().RotateVector({0, 0, 1}).Z(), -1.0, 1.0));
    // Body levelling: slopes (in metres of leg length per metre of body offset) that the
    // leg targets apply along x (nose up) and y (left side up).
    const double noseUp = pose.Rot().RotateVector({1, 0, 0}).Z();
    const double leftUp = pose.Rot().RotateVector({0, 1, 0}).Z();
    levelX_ = std::clamp(levelX_ + levelGain_ * noseUp * dt, -0.6, 0.6);
    levelY_ = std::clamp(levelY_ + levelGain_ * leftUp * dt, -0.6, 0.6);
    const double climb = std::clamp((tilt - tiltStart_) / (tiltFull_ - tiltStart_), 0.0, 1.0);
    const double scale = 1.0 - climb * (1.0 - minScale_);
    vx_ = ramp(vx_, cvx * scale, accel_ * dt);
    vy_ = ramp(vy_, cvy * scale, accel_ * dt);
    wz_ = ramp(wz_, cwz * scale, yawAccel_ * dt);

    // Step frequency: fast enough that no stance stroke exceeds max_stride.
    double maxFootSpeed = 0.0;
    for (const auto & leg : legs_) {
      maxFootSpeed = std::max(maxFootSpeed, std::hypot(vx_ - wz_ * leg.ny, vy_ + wz_ * leg.nx));
    }
    const bool moving = maxFootSpeed > 0.005;
    double freq = std::clamp(maxFootSpeed * duty_ / maxStride_, minFreq_, maxFreq_);

    if (moving) {
      standing_ = false;
    } else if (!standing_) {
      // Finish the steps in progress; stop once both tripods are on the ground.
      const double pa = wrap01(phase_);
      if (pa >= 0.5 && pa < duty_ && settled_) {
        standing_ = true;
      }
    }
    if (!standing_) {
      phase_ = wrap01(phase_ + freq * dt);
    }

    settled_ = true;
    for (auto & leg : legs_) {
      const double ux = vx_ - wz_ * leg.ny;   // body velocity at the foot, body frame
      const double uy = vy_ + wz_ * leg.nx;
      const double p = wrap01(phase_ + 0.5 * leg.group);
      // Legs on the high side get shorter (foot closer to the body), low side longer.
      double fz = -height_ + std::clamp(levelX_ * leg.nx + levelY_ * leg.ny, -levelLimit_, levelLimit_);
      if (standing_ || p < duty_) {
        if (leg.swinging) {
          leg.swinging = false;
        }
        if (!standing_) {
          leg.fx -= ux * dt;
          leg.fy -= uy * dt;
        }
      } else {
        if (!leg.swinging) {
          leg.swinging = true;
          leg.lx = leg.fx;
          leg.ly = leg.fy;
        }
        const double s = (p - duty_) / (1.0 - duty_);
        const double half = 0.5 * duty_ / freq;   // half the next stance stroke, in time
        const double tx = leg.nx + ux * half;
        const double ty = leg.ny + uy * half;
        // Lift (first 25 %), carry forward at full height, drop (last 25 %).
        const double move = smooth((s - 0.2) / 0.6);
        leg.fx = leg.lx + (tx - leg.lx) * move;
        leg.fy = leg.ly + (ty - leg.ly) * move;
        fz += stepHeight_ * (s < 0.5 ? smooth(s / 0.25) : smooth((1.0 - s) / 0.25));
        if (std::hypot(leg.fx - leg.nx, leg.fy - leg.ny) > 0.02) {
          settled_ = false;
        }
      }
      // Keep feet inside the reachable workspace.
      const double dx = leg.fx - leg.nx, dy = leg.fy - leg.ny;
      const double off = std::hypot(dx, dy), lim = 0.75 * maxStride_ + 0.02;
      if (off > lim) {
        leg.fx = leg.nx + dx * lim / off;
        leg.fy = leg.ny + dy * lim / off;
      }

      std::array<double, 3> q;
      Ik(leg, leg.fx, leg.fy, fz, q);
      for (int k = 0; k < 3; ++k) {
        const auto * pos = ecm.Component<components::JointPosition>(leg.joint[k]);
        if (!pos || pos->Data().empty()) {
          continue;
        }
        const double ff = leg.init ? (q[k] - leg.q[k]) / dt : 0.0;
        const double cmd = std::clamp(ff + kp_ * (q[k] - pos->Data()[0]), -maxJointVel_, maxJointVel_);
        auto * vel = ecm.Component<components::JointVelocityCmd>(leg.joint[k]);
        if (vel) {
          vel->Data() = {cmd};
        } else {
          ecm.CreateComponent(leg.joint[k], components::JointVelocityCmd({cmd}));
        }
        leg.q[k] = q[k];
      }
      leg.init = true;
    }
    PublishJointStates(now, ecm);
  }

private:
  struct Leg
  {
    std::array<Entity, 3> joint{kNullEntity, kNullEntity, kNullEntity};
    std::array<std::string, 3> names;
    double mx{0}, my{0}, myaw{0};
    int group{0};
    double nx{0}, ny{0};          // neutral foot position (body frame)
    double fx{0}, fy{0};          // current foot target
    double lx{0}, ly{0};          // lift-off position of the current swing
    bool swinging{false};
    bool init{false};
    std::array<double, 3> q{0, 0, 0};
  };

  void PublishJointStates(double now, const EntityComponentManager & ecm)
  {
    if (!jsPub_ || now - lastJs_ < jsPeriod_) {
      return;
    }
    lastJs_ = now;
    msgs::Model msg;
    const auto sec = static_cast<int64_t>(now);
    msg.mutable_header()->mutable_stamp()->set_sec(sec);
    msg.mutable_header()->mutable_stamp()->set_nsec(static_cast<int32_t>((now - sec) * 1e9));
    for (const auto & leg : legs_) {
      for (int k = 0; k < 3; ++k) {
        const auto * pos = ecm.Component<components::JointPosition>(leg.joint[k]);
        auto * j = msg.add_joint();
        j->set_name(leg.names[k]);
        j->mutable_axis1()->set_position(pos && !pos->Data().empty() ? pos->Data()[0] : 0.0);
      }
    }
    jsPub_.Publish(msg);
  }

  // Foot (body frame) -> coxa, femur, tibia angles. Femur/tibia angles are positive
  // upwards; the tibia angle is relative to the femur (negative = knee bent down).
  void Ik(const Leg & leg, double x, double y, double z, std::array<double, 3> & q) const
  {
    const double c = std::cos(-leg.myaw), s = std::sin(-leg.myaw);
    const double lx = c * (x - leg.mx) - s * (y - leg.my);
    const double ly = s * (x - leg.mx) + c * (y - leg.my);
    q[0] = std::atan2(ly, lx);
    const double r = std::hypot(lx, ly) - l1_;
    const double d = std::clamp(std::hypot(r, z), std::abs(l2_ - l3_) + 1e-3, l2_ + l3_ - 1e-3);
    const double a = std::acos(std::clamp((l2_ * l2_ + d * d - l3_ * l3_) / (2 * l2_ * d), -1.0, 1.0));
    const double g = std::acos(std::clamp((l2_ * l2_ + l3_ * l3_ - d * d) / (2 * l2_ * l3_), -1.0, 1.0));
    q[1] = std::atan2(z, r) + a;
    q[2] = -(M_PI - g);
  }

  void OnTwist(const msgs::Twist & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cmdVx_ = msg.linear().x();
    cmdVy_ = msg.linear().y();
    cmdWz_ = msg.angular().z();
    newCmd_ = true;
  }

  transport::Node node_;
  transport::Node::Publisher jsPub_;
  double jsPeriod_{1.0 / 30.0};
  double lastJs_{-1e9};
  Entity model_{kNullEntity};
  std::vector<Leg> legs_;
  std::mutex mutex_;
  double cmdVx_{0}, cmdVy_{0}, cmdWz_{0};
  bool newCmd_{false};
  double lastCmdTime_{-1e9};

  double l1_, l2_, l3_, height_, reach_, stepHeight_, maxStride_, duty_;
  double minFreq_, maxFreq_, maxSpeed_, maxYawRate_, accel_, yawAccel_;
  double kp_, maxJointVel_, timeout_, tiltStart_, tiltFull_, minScale_, levelGain_, levelLimit_;
  double levelX_{0}, levelY_{0};
  double vx_{0}, vy_{0}, wz_{0}, phase_{0.5};
  bool standing_{true};
  bool settled_{true};
};
}  // namespace rescue_hexabot

IGNITION_ADD_PLUGIN(rescue_hexabot::HexWalk, ignition::gazebo::System,
                    rescue_hexabot::HexWalk::ISystemConfigure,
                    rescue_hexabot::HexWalk::ISystemPreUpdate)
