#include "alpha_control.h"
#include "ros/ros.h"
#include "dictionary.h"
#include "exception.hpp"

AlphaControl::AlphaControl() {

    m_pid = std::make_shared<MimoPID>();

    m_pid->set_error_function(std::bind(&AlphaControl::f_error_function, this, std::placeholders::_1, std::placeholders::_2));

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

Eigen::VectorXf AlphaControl::calculate_needed_forces(float dt) {

    Eigen::VectorXf u;
    f_calculate_pid(u, dt);

    Eigen::VectorXf t;
    if(f_optimize_thrust(t, u)) {
        return t;
    } else {
        ROS_WARN_STREAM("Optimum solution can not be found");
    }

    return Eigen::VectorXf::Zero(m_control_allocation_matrix.cols());
}

void AlphaControl::f_calculate_pid(Eigen::VectorXf &u, double dt) {
    u = m_pid->calculate(m_desired_state, m_system_state, dt);
}

bool AlphaControl::f_optimize_thrust(Eigen::VectorXf &t, Eigen::VectorXf u) {

    // Control allocation matrix
    Eigen::MatrixXf T(m_controlled_freedoms.size(), m_control_allocation_matrix.cols());

    // Control matrix
    Eigen::VectorXf U(m_controlled_freedoms.size());

    for(int i = 0 ; i < m_controlled_freedoms.size() ; i++) {
        T.row(i) = m_control_allocation_matrix.row(m_controlled_freedoms.at(i));
        U(i) = u(m_controlled_freedoms.at(i));
    }

    // Q -> objective matrix
    Eigen::MatrixXf Q = 2 * T.transpose() * T;
    // c -> objective vector
    Eigen::VectorXd c = (-(T.transpose() * U).transpose() - (U.transpose() * T)).transpose().cast<double>();

    std::vector<Eigen::Triplet<double>> Q_triplets;
    for(int i = 0 ; i < Q.rows() ; i++) {
        for(int j = 0 ; j < Q.cols() ; j++) {
            Q_triplets.emplace_back(Eigen::Triplet<double>{i, j, static_cast<double>(Q(i,j))});
        }
    }

    osqp::OsqpInstance qp_instance;
    Eigen::SparseMatrix<double> Q_sparse(Q.rows(), Q.cols());
    Q_sparse.setFromTriplets(Q_triplets.begin(), Q_triplets.end());
    qp_instance.objective_matrix = Q_sparse;

    qp_instance.objective_vector = c;
    Eigen::VectorXd upper_bounds = THRUST_LIMIT_NEWTON * Eigen::VectorXd::Ones(T.cols());
    Eigen::VectorXd lower_bounds = -THRUST_LIMIT_NEWTON * Eigen::VectorXd::Ones(T.cols());
    qp_instance.lower_bounds = lower_bounds;
    qp_instance.upper_bounds = upper_bounds;
    qp_instance.constraint_matrix = Eigen::SparseMatrix<double>(Q.cols(),Q.cols());

    osqp::OsqpSolver solver;
    osqp::OsqpSettings settings;

    settings.verbose = false;

    auto status = solver.Init(qp_instance, settings);

    if(not status.ok()) {
        return false;
    }

    osqp::OsqpExitCode exitCode = solver.Solve();

    switch (exitCode) {
        case osqp::OsqpExitCode::kOptimal:
            t = solver.primal_solution().cast<float>();
            return true;
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

    return false;
}

void AlphaControl::set_controlled_freedoms(decltype(m_controlled_freedoms) f) {
    m_controlled_freedoms = f;
}

Eigen::ArrayXf AlphaControl::f_error_function(Eigen::ArrayXf desired, Eigen::ArrayXf current) {

    if(desired.size() != current.size()) {
        throw control_exception("desired and current state sizes are different");
    }

    Eigen::ArrayXf error = desired - current;

    for(const auto& i : std::vector<int>{
        STATE_ROLL_INDEX,
        STATE_PITCH_INDEX,
        STATE_YAW_INDEX
    }) {
        error(i) = atan2(sin(desired(i) - current(i)), cos(desired(i) - current(i)));
    }

    return error;
}
