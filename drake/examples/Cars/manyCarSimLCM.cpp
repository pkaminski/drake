// #include "drake/systems/LCMSystem.h"
// #include "drake/systems/LinearSystem.h"
// #include "drake/systems/pd_control_system.h"
// #include "drake/systems/plants/BotVisualizer.h"
// #include "drake/systems/plants/RigidBodySystem.h"
// #include "drake/util/drakeAppUtil.h"

// using namespace std;
// using namespace Eigen;
// using namespace Drake;

int main(int argc, const char* const argv[]) {
#if 0
  // todo: consider moving this logic into the RigidBodySystem class so it can
  // be reused
  DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::QUATERNION;

  auto rigid_body_sys = make_shared<RigidBodySystem>();
  rigid_body_sys->addRobotFromFile(argv[1], floating_base_type);
  auto const & tree = rigid_body_sys->getRigidBodyTree();
  for (int i=2; i<argc; i++)
    tree->addRobotFromSDF(argv[i], DrakeJoint::FIXED);  // add environment

  if (true) {  // add flat terrain
    double box_width = 1000;
    double box_depth = 10;
    DrakeShapes::Box geom(Vector3d(box_width, box_width, box_depth));
    Isometry3d T_element_to_link = Isometry3d::Identity();
    T_element_to_link.translation() << 0, 0,
        -box_depth / 2;  // top of the box is at z=0
    auto& world = tree->bodies[0];
    Vector4d color;
    color << 0.9297, 0.7930, 0.6758,
        1;  // was hex2dec({'ee','cb','ad'})'/256 in matlab
    world->addVisualElement(
        DrakeShapes::VisualElement(geom, T_element_to_link, color));
    tree->addCollisionElement(
        RigidBody::CollisionElement(geom, T_element_to_link, world), *world,
        "terrain");
    tree->updateStaticCollisionElements();
  }

  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();
  auto visualizer =
      make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);
  auto sys = cascade(vehicle_sys, visualizer);

  SimulationOptions options = default_simulation_options;
  options.timeout_seconds = numeric_limits<double>::infinity();

  VectorXd x0 = VectorXd::Zero(rigid_body_sys->getNumStates());
  x0.head(tree->num_positions) = tree->getZeroConfiguration();
  runLCM(sys, lcm, 0, std::numeric_limits<double>::infinity(), x0, options);
#endif

  return 0;
}
