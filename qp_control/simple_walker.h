#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace qp_control {

template<typename T>
class simple_walker : public systems::LeafSystem<T>
{
private:
    int visualizerPortIndex;
    void VisualizerStateOut(const systems::Context<T>& context, systems::BasicVector<T>* output) const;
    void DoCalcTimeDerivatives(const systems::Context<T>& context, systems::ContinuousState<T>* derivatives) const;
public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(simple_walker);
    simple_walker();
    const systems::OutputPort<T>& get_visualizer_port() {
        return this->get_output_port(visualizerPortIndex);
    }
};

}
}