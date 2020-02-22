#include "drake/qp_control/qp_controller.h"

namespace drake {
namespace qp_control {

template<typename T>
QPController<T>::QPController(std::unique_ptr<RigidBodyTree<T>> tree_) {
    tree = std::move(tree_);
    estimatedStatePort = this->DeclareVectorInputPort("estimated state", systems::BasicVector<T>(24)).get_index();
    desiredStatePort = this->DeclareVectorInputPort("desired state", systems::BasicVector<T>(24)).get_index();
    controlOutputPort = this->DeclareVectorOutputPort("u", systems::BasicVector<T>(6), &QPController::CalcControlOutput).get_index();
}

template<typename T>
void QPController<T>::CalcControlOutput(const systems::Context<T>& context, systems::BasicVector<T>* output) const {
    tree->
    output->SetAtIndex(0, 1);
}

template class QPController<double>;

}
}