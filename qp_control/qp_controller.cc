#include "drake/qp_control/qp_controller.h"

namespace drake {
namespace qp_control {

template<typename T>
QPController<T>::QPController(std::unique_ptr<RigidBodyTree<T>> tree_) {
    tree = tree_.release();
    estimatedStatePort = this->DeclareVectorInputPort("estimated state", systems::BasicVector<T>(24)).get_index();
    desiredStatePort = this->DeclareVectorInputPort("desired state", systems::BasicVector<T>(24)).get_index();
    controlOutputPort = this->DeclareVectorOutputPort("u", systems::BasicVector<T>(6), &QPController::CalcControlOutput).get_index();

    this->DeclareDiscreteState(6);
    this->DeclarePeriodicDiscreteUpdateEvent(0.0005, 0, &QPController::DiscreteUpdate);
}

template<typename T>
void QPController<T>::CalcControlOutput(const systems::Context<T>& context, 
                                        systems::BasicVector<T>* output) const {
    //systems::System<T>::get_input_port(estimatedStatePort).Eval(context)
    // output->SetFromVector(VectorX<double>::Ones(6));
    // auto kinSol = tree->doKinematics(systems::System<T>::get_input_port(estimatedStatePort).Eval(context));
    output->SetFromVector(context.get_discrete_state_vector().get_value());
    // output->SetFromVector(VectorX<double>::Zero(6));
}

template<typename T>
void QPController<T>::DiscreteUpdate(const systems::Context<double>& context,
                                     systems::DiscreteValues<double>* state) const {
    VectorX<double> x = systems::System<double>::EvalVectorInput(context, 0)->CopyToVector();
    VectorX<double> q(NQ), qd(NV);

    for(int i = 0; i < NQ; i++) {
        q[i] = x[i];
        qd[i] = x[i+NQ];
    }
    auto kinsol = tree->doKinematics(q, qd);

    /*
    Variables
    */
    auto invDynamicsSolution = tree->inverseDynamics(kinsol,{},qd, true);
    
    // Eigen::MatrixXd A, B, C, D_t0, D_t1, Q, R;
    // A << 0, 0, 1, 0,
    //      0, 0, 0, 1,
    //      0, 0, 0, 0,
    //      0, 0, 0, 0;
    // B << 0, 0,
    //      0, 0,
    //      1, 0,
    //      0, 1;
    // C << 1, 0, 0, 0,
    //      0, 1, 0, 0;
    // D_t0 = -(q[2])/(-9.8) * Eigen::MatrixXd::Identity(2,2);
    // D_t1 = -(q[2])/(-9.8) * Eigen::MatrixXd::Identity(2,2);

    // Q = Eigen::MatrixXd::Identity(4,2);
    // R = Eigen::MatrixXd::Zero(1);

    // auto lqr_result = systems::controllers::LinearQuadraticRegulator(D_t1-D_t0, , Q, R);

    state->get_mutable_vector().SetFromVector(invDynamicsSolution.head(6));
}

template class QPController<double>;

}
}