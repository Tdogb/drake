#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace qp_control {

template<typename T>
class simple_walker : public systems::LeafSystem<T>
{
private:
    // int visualizerPort;
    int stateOutPort;
    void StateOut(const systems::Context<T>& context, systems::BasicVector<T>* output) const;
    void VisualizerStateOut(const systems::Context<T>& context, systems::BasicVector<T>* output) const;
    void DoCalcTimeDerivatives(const systems::Context<T>& context, systems::ContinuousState<T>* derivatives) const;
public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(simple_walker);
    simple_walker();
    // const systems::InputPort<T>& get_visualizer_port() {
    //     return this->get_input_port(visualizerPort);
    // }
    const systems::OutputPort<T>& get_state_out_port() {
        return this->get_output_port(stateOutPort);
    }
};

}
}