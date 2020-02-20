#include "drake/qp_control/simple_walker.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/common/find_resource.h"

namespace drake {
namespace qp_control {
    int doMain() {
        systems::DiagramBuilder<double> builder;
        auto walker = builder.AddSystem<simple_walker<double>>();
        walker->set_name("simple_walker");

        lcm::DrakeLcm lcm;
        auto tree = std::make_unique<RigidBodyTree<double>>();
        parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        FindResourceOrThrow("drake/qp_control/CompassGait.urdf"), multibody::joints::kRollPitchYaw, tree.get());
        
        DrakeShapes::Box geom(Eigen::Vector3d(100, 1, 10));
        Eigen::Isometry3d X_WB = Eigen::Isometry3d::Identity();
        Eigen::Vector4d color(0.9297, 0.7930, 0.6758, 1);
        RigidBody<double>& world = tree->world();
        world.AddVisualElement(DrakeShapes::VisualElement(geom, X_WB, color));
        tree->addCollisionElement(drake::multibody::collision::Element(geom, X_WB, &world),
                                  world,
                                  "terrain");
        tree->compile();

        auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
        publisher->set_name("publisher");
        builder.Connect(walker->get_visualizer_port(),
                        publisher->get_input_port(0));

        auto diagram = builder.Build();
        auto context = diagram->AllocateContext();
        systems::VectorBase<double>& cstate = context->get_mutable_continuous_state_vector();
        cstate.SetAtIndex(0, 0.0);

        systems::Simulator<double> simulator(*diagram, std::move(context));
        // systems::Context<double>& rw_context = diagram->GetMutableSubsystemContext(*walker, &simulator.get_mutable_context());
        simulator.Initialize();
        simulator.set_target_realtime_rate(1.0);
        simulator.get_mutable_context().SetAccuracy(1e-4);
        simulator.AdvanceTo(1);

        return 0;
    }
}
}

int main(int argc, char* argv[]) {
    argc = 0;
    argv = nullptr;
    return drake::qp_control::doMain();
}

