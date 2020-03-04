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
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/sine.h"

namespace drake {
namespace qp_control {
    int doMain() {
        manipulation::util::SimDiagramBuilder<double> builder;
        systems::RigidBodyPlant<double>* plant = nullptr;
        systems::controllers::StateFeedbackControllerInterface<double>* controller = nullptr;

        auto builder_base = builder.get_mutable_builder();

        lcm::DrakeLcm lcm;
        auto tree = std::make_unique<RigidBodyTree<double>>();
        
        parsers::urdf::AddModelInstanceFromUrdfFileToWorld(FindResourceOrThrow("drake/qp_control/KneedCompassGait.urdf"), multibody::joints::kRollPitchYaw, tree.get());
        
        drake::multibody::AddFlatTerrainToWorld(tree.get(), 100., 10.);
        tree->compile();
        plant = builder.AddPlant(std::move(tree));
        {
        const double kYoung = 1e8; // Pa
        const double kDissipation = 5.0; // s/m
        const double kStaticFriction = 10;
        const double kDynamicFriction = 10;
        drake::systems::CompliantMaterial default_material;
        default_material.set_youngs_modulus(kYoung);
        default_material.set_dissipation(kDissipation);
        default_material.set_friction(kStaticFriction, kDynamicFriction);
        plant->set_default_compliant_material(default_material);

        const double kStictionSlipTolerance = 0.01; // m/s
        const double kContactRadius = 2e-3; //m
        drake::systems::CompliantContactModelParameters model_parameters;
        model_parameters.characteristic_radius = kContactRadius;
        model_parameters.v_stiction_tolerance = kStictionSlipTolerance;

        plant->set_contact_model_parameters(model_parameters);
        }
        controller = builder.AddController<QPController<double>>(RigidBodyTreeConstants::kFirstNonWorldModelInstanceId, plant->get_rigid_body_tree().Clone());
        auto desired_state = builder_base->AddSystem<systems::ConstantVectorSource<double>>(Eigen::VectorXd::Ones(24));

        builder.AddVisualizer(&lcm);
        builder.get_visualizer()->set_publish_period(0.005);

        builder_base->AddSystem<systems::Sine<double>>(0.25, 0.5, 0, 1);

        builder_base->Connect(desired_state->get_output_port(),
                              controller->get_input_port_desired_state());

        auto diagram = builder.Build();

        systems::Simulator<double> simulator(*diagram); //, std::move(context)
        simulator.set_target_realtime_rate(1.0);
        simulator.set_publish_every_time_step(false);
        simulator.get_mutable_context().SetAccuracy(1e-4);
        VectorX<double> zeroState(24);
        zeroState << -0.9504, 0, 0.95, 0, 0, 0, 0, 0, 0.3300, -1.01, 0.7353, -0.9077, -0.9, 0.30, 0.1245, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        plant->set_state_vector(&simulator.get_mutable_context(), zeroState);
        simulator.Initialize();
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

