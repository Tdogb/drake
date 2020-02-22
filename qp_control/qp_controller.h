#pragma once

#include <memory>
#include <string>

#include "drake/solvers/mathematical_program.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/controllers/state_feedback_controller_interface.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace qp_control {

using solvers::MathematicalProgram;

template<typename T>
class QPController : public systems::LeafSystem<T>, 
                     public systems::controllers::StateFeedbackControllerInterface<T>
{
private:
    MathematicalProgram prog;
    std::unique_ptr<RigidBodyTree<T>> tree;
    int estimatedStatePort;
    int desiredStatePort;
    int controlOutputPort;
    void CalcControlOutput(const systems::Context<T>& context, systems::BasicVector<T>* output) const;
public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QPController);
    QPController(std::unique_ptr<RigidBodyTree<T>> tree_);

    const systems::InputPort<T>& get_input_port_estimated_state() const override {
        return systems::Diagram<T>::get_input_port(0);
    }

    const systems::InputPort<T>& get_input_port_desired_state() const override {
        return systems::Diagram<T>::get_input_port(0);
    }

    const systems::OutputPort<T>& get_output_port_control() const override {
        return systems::Diagram<T>::get_output_port(0);
    }
};
    
}
}