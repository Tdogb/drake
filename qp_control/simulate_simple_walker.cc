#include "drake/qp_control/simple_walker.h"
#include "drake/qp_control/qp_controller.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/common/find_resource.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include <drake/systems/controllers/pid_controller.h>
#include "drake/systems/controllers/state_feedback_controller_interface.h"

namespace drake {
namespace qp_control {
    int doMain() {
        manipulation::util::SimDiagramBuilder<double> builder;
        systems::RigidBodyPlant<double>* plant = nullptr;
        systems::controllers::StateFeedbackControllerInterface<double>* controller = nullptr;

        auto builder_base = builder.get_mutable_builder();
        auto walker = builder_base->AddSystem<simple_walker<double>>();
        walker->set_name("simple_walker");

        lcm::DrakeLcm lcm;
        auto tree = std::make_unique<RigidBodyTree<double>>();
        
        parsers::urdf::AddModelInstanceFromUrdfFileToWorld(FindResourceOrThrow("drake/qp_control/KneedCompassGait.urdf"), multibody::joints::kRollPitchYaw, tree.get());
        
        drake::multibody::AddFlatTerrainToWorld(tree.get(), 100., 10.);
        tree->compile();
        plant = builder.AddPlant(std::move(tree));
        controller = builder.AddController<QPController<double>>(RigidBodyTreeConstants::kFirstNonWorldModelInstanceId, plant->get_rigid_body_tree().Clone());

        builder.AddVisualizer(&lcm);
        // builder.get_visualizer()->set_publish_period(1e-2);

        auto diagram = builder.Build();
        // auto context = diagram->AllocateContext();
        // systems::VectorBase<double>& cstate = context->get_mutable_continuous_state_vector();
        // cstate.SetAtIndex(0, 0.0);
        // cstate.SetAtIndex(1, 1.0);
        // cstate.SetAtIndex(2, 5.0);
        // cstate.SetAtIndex(3, 0.0);

        systems::Simulator<double> simulator(*diagram); //, std::move(context)
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

