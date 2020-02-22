#include "drake/qp_control/simple_walker.h"

namespace drake {
namespace qp_control {

template<typename T>
simple_walker<T>::simple_walker(/* args */)
{
    this->DeclareVectorInputPort("torque", systems::BasicVector<double>(1));
    this->DeclareContinuousState(0,0,4);
    stateOutPort = this->DeclareVectorOutputPort("state out", systems::BasicVector<T>(2), &simple_walker::StateOut).get_index();
    // visualizerPort = this->DeclareVectorOutputPort("visualizerStateOut", systems::BasicVector<T>(14), &simple_walker::VisualizerStateOut).get_index(); //For visualizer, includes derrivatives
}

// template<typename T>
// void simple_walker<T>::VisualizerStateOut(const systems::Context<T>& context, systems::BasicVector<T>* output) const {
//     const auto cstate = context.get_continuous_state().CopyToVector();
//     std::cout << cstate.size() << std::endl;
//     output->SetAtIndex(0, cstate[0]);
// }

template<typename T>
void simple_walker<T>::StateOut(const systems::Context<T>& context, systems::BasicVector<T>* output) const {
    output->SetAtIndex(0, 2);
}


template<typename T>
void simple_walker<T>::DoCalcTimeDerivatives(const systems::Context<T>& context, systems::ContinuousState<T>* derivatives) const {

}

template class simple_walker<double>;

}
}