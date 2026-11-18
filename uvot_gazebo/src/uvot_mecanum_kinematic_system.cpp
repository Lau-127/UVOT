#include <gz/plugin/Register.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

#include <gz/math/Vector3.hh>

#include <memory>
#include <string>
#include <vector>

namespace uvot_gazebo
{

class UVOTMecanumKinematicSystem:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);

    if (!this->model.Valid(_ecm))
    {
      gzerr << "[UVOTMecanumKinematicSystem] Plugin must be attached to a model.\n";
      return;
    }

    // Parámetros geométricos
    if (_sdf->HasElement("wheel_radius"))
      this->wheelRadius = _sdf->Get<double>("wheel_radius");

    if (_sdf->HasElement("lx"))
      this->lx = _sdf->Get<double>("lx");

    if (_sdf->HasElement("ly"))
      this->ly = _sdf->Get<double>("ly");

    if (_sdf->HasElement("base_link"))
      this->baseLinkName = _sdf->Get<std::string>("base_link");

    // Joints:
    // wheel1 = rear-left
    // wheel2 = rear-right
    // wheel3 = front-right
    // wheel4 = front-left
    const auto w1Entity = this->model.JointByName(_ecm, "wheel1_joint");
    const auto w2Entity = this->model.JointByName(_ecm, "wheel2_joint");
    const auto w3Entity = this->model.JointByName(_ecm, "wheel3_joint");
    const auto w4Entity = this->model.JointByName(_ecm, "wheel4_joint");

    if (w1Entity == gz::sim::kNullEntity ||
        w2Entity == gz::sim::kNullEntity ||
        w3Entity == gz::sim::kNullEntity ||
        w4Entity == gz::sim::kNullEntity)
    {
      gzerr << "[UVOTMecanumKinematicSystem] Could not find wheel joints.\n";
      return;
    }

    this->wheel1 = gz::sim::Joint(w1Entity);
    this->wheel2 = gz::sim::Joint(w2Entity);
    this->wheel3 = gz::sim::Joint(w3Entity);
    this->wheel4 = gz::sim::Joint(w4Entity);

    this->wheel1.EnableVelocityCheck(_ecm);
    this->wheel2.EnableVelocityCheck(_ecm);
    this->wheel3.EnableVelocityCheck(_ecm);
    this->wheel4.EnableVelocityCheck(_ecm);

    const auto baseEntity =
      this->model.LinkByName(_ecm, this->baseLinkName);

    if (baseEntity == gz::sim::kNullEntity)
    {
      gzerr << "[UVOTMecanumKinematicSystem] Could not find base link ["
            << this->baseLinkName << "].\n";
      return;
    }

    this->baseLink = gz::sim::Link(baseEntity);
    this->configured = true;
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->configured || _info.paused)
      return;

    const auto v1 = this->wheel1.Velocity(_ecm);
    const auto v2 = this->wheel2.Velocity(_ecm);
    const auto v3 = this->wheel3.Velocity(_ecm);
    const auto v4 = this->wheel4.Velocity(_ecm);

    if (!v1 || !v2 || !v3 || !v4)
      return;

    if (v1->empty() || v2->empty() || v3->empty() || v4->empty())
      return;

    const double w1 = (*v1)[0];
    const double w2 = (*v2)[0];
    const double w3 = (*v3)[0];
    const double w4 = (*v4)[0];

    /*
     * Tu disposición:
     *
     * wheel4 = front-left
     * wheel3 = front-right
     * wheel1 = rear-left
     * wheel2 = rear-right
     *
     * Patrón observado con mecanum_drive_controller:
     *
     * wz > 0:
     * wheel1 = -
     * wheel2 = +
     * wheel3 = +
     * wheel4 = -
     *
     * La formulación se adapta a esos signos.
     */

    const double r = this->wheelRadius;
    const double k = this->lx + this->ly;

    const double vx =
      r * 0.25 * (w1 + w2 + w3 + w4);

    // vy > 0 = hacia la izquierda (REP-103). Inversa exacta de
    // mecanum_drive_controller: FL = vx - vy - k wz, FR = vx + vy + k wz,
    // RL = vx + vy - k wz, RR = vx - vy + k wz  ->  vy = r/4 (RL - RR + FR - FL)
    // (la versión anterior tenía el signo cambiado y movía el robot a la derecha)
    const double vy =
      r * 0.25 * (w1 - w2 + w3 - w4);

    const double wz =
      r / (4.0 * k) * (-w1 + w2 + w3 - w4);

    // SetLinearVelocity / SetAngularVelocity reciben velocidades
    // expresadas en el frame local del link.
    this->baseLink.SetLinearVelocity(
      _ecm,
      gz::math::Vector3d(vx, vy, 0.0));

    this->baseLink.SetAngularVelocity(
      _ecm,
      gz::math::Vector3d(0.0, 0.0, wz));
  }

private:
  gz::sim::Model model{gz::sim::kNullEntity};

  gz::sim::Joint wheel1;
  gz::sim::Joint wheel2;
  gz::sim::Joint wheel3;
  gz::sim::Joint wheel4;

  gz::sim::Link baseLink;

  double wheelRadius{0.04};

  // Distancia desde el centro del robot hasta cada rueda
  // en x e y.
  double lx{0.08125};
  double ly{0.105};

  std::string baseLinkName{"robot1/base_footprint"};

  bool configured{false};
};

}  // namespace uvot_gazebo


GZ_ADD_PLUGIN(
  uvot_gazebo::UVOTMecanumKinematicSystem,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  uvot_gazebo::UVOTMecanumKinematicSystem,
  "uvot_gazebo::UVOTMecanumKinematicSystem")
