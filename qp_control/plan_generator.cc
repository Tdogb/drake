// #include "drake/qp_control/plan_generator.h"

// namespace drake {
// namespace qp_control {

// template<typename T>
// PlanGenerator<T>::PlanGenerator() {
//     this->DeclareDiscreteState(3,3,0);
//     this->DeclarePeriodicUpdateEvent();
//     this->DeclareVectorOutputPort("swing foot traj", systems::BasicVector<T>(3), );
//     this->DeclareVectorOutputPort("com traj", systems::BasicVector<T>(9), );
//     this->DeclareVectorOutputPort("swing foot", systems::BasicVector<T>(1), )
// }

// // template<typename T>
// // void PlanGenerator<T>::DoCalcTimeDerivatives(systems::Context<T>& context, const systems::BasicVector<T>* output) const  {

// // }

// template<typename T>
// void PlanGenerator<T>::DoDiscreteUpdateEvent(const systems::Context<T>& context, systems::DiscreteValues<T>* values) const {

// }

// }
// }