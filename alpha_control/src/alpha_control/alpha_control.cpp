#include "alpha_control.h"
#include "ros/ros.h"
#include "dictionary.h"
#include "exception.hpp"

AlphaControl::AlphaControl() {

    /**
     * Initialize the error state
     */
    m_error_state = Eigen::VectorXd(STATE_VECTOR_SIZE);

    /**
     * Create the multiple input multiple output PID controller
     */
    m_pid = boost::make_shared<MimoPID>();

    /**
     * MIMO PID object does not implement the error function. It asks programmer
     * to assign a error function. When it calculates the gains it uses the
     * function that is binded to it's error function.
     */
    m_pid->set_error_function(
        boost::bind(
            &AlphaControl::f_error_function,
            this,
            boost::placeholders::_1,
            boost::placeholders::_2
        )
    );

}

void AlphaControl::set_control_allocation_matrix(
        const decltype(m_control_allocation_matrix)& matrix) {
    m_control_allocation_matrix = matrix;
}

auto AlphaControl::get_control_allocation_matrix() ->
        decltype(m_control_allocation_matrix) {
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

void AlphaControl::set_system_state(
        const decltype(m_system_state) &system_state) {
    m_system_state = system_state;
}

auto AlphaControl::get_desired_state() -> decltype(m_desired_state) {
    return m_desired_state;
}

void AlphaControl::set_desired_state(
        const decltype(m_desired_state) &desired_state) {
    m_desired_state = desired_state;
}

bool AlphaControl::calculate_needed_forces(Eigen::VectorXd *f, double dt) {

    /**
     * This function basically computes one iteration of the controller. It
     * computes the PID gains. Optimizes the thrust.
     */

    /**
     * vector 'u' represents the input values of the system. That is all degrees
     * of freedoms. Size of the vector will change based on controllers active
     * mode.
     */
    Eigen::VectorXd u;
    if(!f_calculate_pid(&u, dt)){
        return false;
    }

    /**
     * Below code computes the forces that later will be requested from the
     * thrusters. Values in the force vector are not thruster set points yet.
     */
    if(f_optimize_thrust(f, u)) {
        return true;
    } else {
        ROS_WARN_STREAM("Optimum solution can not be found");
    }

    return false;
}

bool AlphaControl::f_calculate_pid(Eigen::VectorXd *u, double dt) {
    return m_pid->calculate(u, m_desired_state, m_system_state, dt);
}

bool AlphaControl::f_optimize_thrust(Eigen::VectorXd *t, Eigen::VectorXd u) {

    // Control allocation matrix
    Eigen::MatrixXd T(
        m_controlled_freedoms.size(),
        m_control_allocation_matrix.cols()
    );

    // Control matrix
    Eigen::VectorXd U(m_controlled_freedoms.size());

    {
        /**
         * Quadatic equation solving is probably the most time consuming part
         * of the program. During that time changes may happen. Most likely
         * change would happen on the 'm_controlled_freedoms' vector. This
         * vector updates when controllers mode change. Scoped lock is making
         * sure that those variables will not change while execution is inside
         * in this scope.
         */
        boost::recursive_mutex::scoped_lock lock_a(m_allocation_matrix_lock);
        boost::recursive_mutex::scoped_lock lock_b(m_controlled_freedoms_lock);

        /**
         * We are using a portion of the control allication matrix and the
         * control input. This is important for online mode updates.
         *
         */
        for (int i = 0; i < m_controlled_freedoms.size(); i++) {
            T.row(i) = m_control_allocation_matrix.row(
                m_controlled_freedoms.at(i));
            U(i) = u(m_controlled_freedoms.at(i));
        }
    }

    /**
     * Now we are preparing data for quadratic solver to consume.
     */

    // Q -> objective matrix
    Eigen::MatrixXd Q = 2 * T.transpose() * T;
    // c -> objective vector
    Eigen::VectorXd c =
        (-(T.transpose() * U).transpose() - (U.transpose() * T)).transpose();

    std::vector<Eigen::Triplet<double>> Q_triplets;
    for(int i = 0 ; i < Q.rows() ; i++) {
        for(int j = 0 ; j < Q.cols() ; j++) {
            Q_triplets.emplace_back(Eigen::Triplet<double>{i, j, Q(i,j)});
        }
    }

    // Creating a quadratic solver instance.
    osqp::OsqpInstance qp_instance;

    /**
     * Translating the objective matrix into sparse matrix. Eigen::SparseMatrix
     * is the data type that is consumed by quadratic solver.
     */
    Eigen::SparseMatrix<double> Q_sparse(Q.rows(), Q.cols());
    Q_sparse.setFromTriplets(Q_triplets.begin(), Q_triplets.end());
    qp_instance.objective_matrix = Q_sparse;

    qp_instance.objective_vector = c;
    Eigen::VectorXd upper_bounds =
        THRUST_LIMIT_NEWTON * Eigen::VectorXd::Ones(T.cols());

    Eigen::VectorXd lower_bounds =
        -THRUST_LIMIT_NEWTON * Eigen::VectorXd::Ones(T.cols());

    qp_instance.lower_bounds = lower_bounds;

    qp_instance.upper_bounds = upper_bounds;

    qp_instance.constraint_matrix =
        Eigen::SparseMatrix<double>(Q.cols(),Q.cols());

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
            *t = solver.primal_solution();
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

auto AlphaControl::get_state_error() -> decltype(this->m_error_state) {
    return m_error_state;
}

Eigen::ArrayXd AlphaControl::f_error_function(
        Eigen::ArrayXd desired, Eigen::ArrayXd current) {
    boost::recursive_mutex::scoped_lock lock(m_desired_state_lock);

    if(desired.size() != current.size()) {
        throw control_exception(
            "desired and current state sizes are different"
        );
    }

    Eigen::ArrayXd error = desired - current;

    for(const auto& i : std::vector<int>{
        STATE_ROLL_INDEX,
        STATE_PITCH_INDEX,
        STATE_YAW_INDEX
    }) {

        // todo: wrap2pi implementation

        auto d = (fmod(desired(i) + M_PI, 2*M_PI) - M_PI);
        auto c = (fmod(current(i) + M_PI, 2*M_PI) - M_PI);

        auto t = d - c;
        double diff = (fmod(t + M_PI, 2*M_PI) - M_PI);
        error(i) = diff < -M_PI ? diff + 2*M_PI : diff;
    }

    m_error_state = error;

    return error;
}

void AlphaControl::update_control_allocation_matrix(
        const decltype(m_control_allocation_matrix) &m) {
    boost::recursive_mutex::scoped_lock lock(m_allocation_matrix_lock);
    m_control_allocation_matrix = m;
}

void AlphaControl::update_freedoms(std::vector<int> freedoms) {
    boost::recursive_mutex::scoped_lock lock(m_controlled_freedoms_lock);
    m_controlled_freedoms = std::move(freedoms);

}

void AlphaControl::update_desired_state(
        const decltype(m_desired_state) &desired_state) {
    boost::recursive_mutex::scoped_lock lock(m_desired_state_lock);
    m_desired_state = desired_state;
}