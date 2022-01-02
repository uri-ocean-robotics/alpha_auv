#include "alpha_control.h"
#include "ros/ros.h"

AlphaControl::AlphaControl() {

    m_pid = std::make_shared<MimoPID>();

}

void AlphaControl::set_control_allocation_matrix(const decltype(m_control_allocation_matrix)& matrix) {
    m_control_allocation_matrix = matrix;
}

auto AlphaControl::get_control_allocation_matrix() -> decltype(m_control_allocation_matrix) {
    return m_control_allocation_matrix;
}

auto AlphaControl::get_pid() -> decltype(m_pid) {
    return m_pid;
}

void AlphaControl::set_pid(const MimoPID::Ptr &pid) {
    m_pid = pid;
}

auto AlphaControl::get_system_state() -> decltype(m_system_state) {
    return m_system_state;
}

void AlphaControl::set_system_state(const decltype(m_system_state) &system_state) {
    m_system_state = system_state;
}

auto AlphaControl::get_desired_state() -> decltype(m_desired_state) {
    return m_desired_state;
}

void AlphaControl::set_desired_state(const decltype(m_desired_state) &desired_state) {
    m_desired_state = desired_state;
}

Eigen::VectorXf AlphaControl::calculate_setpoints(float dt) {

    Eigen::VectorXf u = m_pid->calculate(m_desired_state, m_system_state, dt);

    // Q -> objective matrix
    // TODO: objective matrix doesn't change at each iteration. Computing it in every iteration is an absolute waste-of-a-cycle-time.
    Eigen::MatrixXf Q = 2 * m_control_allocation_matrix.transpose() * m_control_allocation_matrix;

    std::vector<Eigen::Triplet<float>> Q_triplets;
    for(int i = 0 ; i < Q.rows() ; i++) {
        for(int j = 0 ; j < Q.cols() ; j++) {
            Q_triplets.emplace_back(Eigen::Triplet<float>{i, j, Q(i,j)});
        }
    }

    // c -> objective vector
    Eigen::MatrixXf c = (-(m_control_allocation_matrix.transpose() * u).transpose() - (u.transpose() * m_control_allocation_matrix)).transpose();

    auto limit_u = THRUST_LIMIT_NEWTON * Eigen::VectorXd::Ones(m_control_allocation_matrix.cols());

    auto limit_l = -THRUST_LIMIT_NEWTON * Eigen::VectorXd::Ones(m_control_allocation_matrix.cols());

    osqp::OsqpInstance qp_instance;

    qp_instance.objective_matrix.setFromTriplets(Q_triplets.begin(), Q_triplets.end());
    qp_instance.objective_vector = c.cast<double>();
    qp_instance.lower_bounds = limit_l;
    qp_instance.upper_bounds = limit_u;

    osqp::OsqpSolver solver;

    osqp::OsqpSettings settings;

    auto status = solver.Init(qp_instance, settings);

    if(status.ok()) {
        osqp::OsqpExitCode exitCode = solver.Solve();

        Eigen::VectorXf optimal_solution;

        switch (exitCode) {
            case osqp::OsqpExitCode::kOptimal:
                optimal_solution = solver.primal_solution().cast<float>();
                return optimal_solution;
                break;
            case osqp::OsqpExitCode::kPrimalInfeasible:
                ROS_WARN_STREAM("kPrimalInfeasible");
                break;
            case osqp::OsqpExitCode::kDualInfeasible:
                ROS_WARN_STREAM("kDualInfeasible");
                break;
            case osqp::OsqpExitCode::kOptimalInaccurate:
                ROS_WARN_STREAM("kOptimalInaccurate");
                break;
            case osqp::OsqpExitCode::kPrimalInfeasibleInaccurate:
                ROS_WARN_STREAM("kPrimalInfeasibleInaccurate");
                break;
            case osqp::OsqpExitCode::kDualInfeasibleInaccurate:
                ROS_WARN_STREAM("kDualInfeasibleInaccurate");
                break;
            case osqp::OsqpExitCode::kMaxIterations:
                ROS_WARN_STREAM("kMaxIterations");
                break;
            case osqp::OsqpExitCode::kInterrupted:
                ROS_WARN_STREAM("kInterrupted");
                break;
            case osqp::OsqpExitCode::kTimeLimitReached:
                ROS_WARN_STREAM("kTimeLimitReached");
                break;
            case osqp::OsqpExitCode::kNonConvex:
                ROS_WARN_STREAM("kNonConvex");
                break;
            case osqp::OsqpExitCode::kUnknown:
                ROS_WARN_STREAM("kUnknown");
                break;
            default:
                break;
        }

    }


    return Eigen::VectorXf::Zero(m_control_allocation_matrix.cols());
}