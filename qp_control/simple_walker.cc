#include "drake/qp_control/simple_walker.h"

namespace drake {
namespace qp_control {

template<typename T>
simple_walker<T>::simple_walker(/* args */)
{
    visualizerPortIndex = this->DeclareVectorInputPort("torque", systems::BasicVector<double>(1)).get_index();
    this->DeclareContinuousState(0,0,4);
    this->DeclareVectorOutputPort("visualizerStateOut", systems::BasicVector<double>(14), &simple_walker::VisualizerStateOut); //For visualizer, includes derrivatives
}

template<typename T>
void simple_walker<T>::VisualizerStateOut(const systems::Context<T>& context, systems::BasicVector<T>* output) const {
    const auto cstate = context.get_continuous_state().CopyToVector();
    output->SetAtIndex(1, cstate[0]);
}

template<typename T>
void simple_walker<T>::DoCalcTimeDerivatives(const systems::Context<T>& context, systems::ContinuousState<T>* derivatives) const {

}


template class simple_walker<double>;

}
}