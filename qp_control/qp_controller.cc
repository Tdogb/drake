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
    this->DeclarePeriodicDiscreteUpdateEvent(0.001, 0, &QPController::DiscreteUpdate);
    {
    const double alpha = 1000;
    auto vdot = prog.NewContinuousVariables(NV, "joint acceleration"); // qddot var
    auto lambda = prog.NewContinuousVariables(NF, "friction forces"); // contact force var
    auto u = prog.NewContinuousVariables(NU, "input");  // torque var
    auto yita = prog.NewContinuousVariables(3, "stance leg relax"); // 3 stance leg slack variables
    vdot.evaluator();
    /** Size of contact Jacobian * 3 **/
    Eigen::Matrix<double, 3, 24> A_contactJacobian;
    Eigen::Matrix<double, 
    prog.AddLinearEqualityConstraint( {vdot, vita});
    }
}

template<typename T>
void QPController<T>::CalcControlOutput(const systems::Context<T>& context, 
                                        systems::BasicVector<T>* output) const {
    output->SetFromVector(context.get_discrete_state_vector().get_value());
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

    // std::cout << tree->transformPointsJacobianDotTimesV(kinsol, Eigen::Vector3d::Zero(3), tree->FindBodyIndex("left_lower_leg"), 0) << std::endl;
    // std::cout << "0-------0" << std::endl;
    // auto invDynamicsSolution = tree->inverseDynamics(kinsol,{},qd, true);
    
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

    // state->get_mutable_vector().SetFromVector(invDynamicsSolution.head(6));
    state->get_mutable_vector().SetFromVector(0*Eigen::VectorXd::Ones(6));
}

template class QPController<double>;

}
}