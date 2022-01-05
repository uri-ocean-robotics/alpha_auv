#ifndef ALPHA_CONTROL_POLYNOMIAL_SOLVER_H
#define ALPHA_CONTROL_POLYNOMIAL_SOLVER_H

#include "vector"
#include "complex"
#include "memory"
#include "gsl/gsl_poly.h"

/** @brief Polynomial Solver
 *
 * Take a polynomial defined as
 *   f(x) = a_n x^n + a_(n-1) x^(n-1) + ... + a_1 x + a_0
 *
 * Class #PolynomialSolver finds the roots of that polynomial
 */
class PolynomialSolver {
private:

    /** @brief Coefficient vector
     *
     * nth variable of the vector describes the coefficient for nth degree
     *
     * @example {-1, 0, 0, 0, 1} Represents f(x) = x^4 - 1
     */
    std::vector<double> m_coeff;

public:
    PolynomialSolver() = default;

    explicit PolynomialSolver(std::vector<double> coeff);

    void set_coeff(decltype(m_coeff) coeff);

    auto get_coeff() -> decltype(m_coeff);

    bool solve(std::vector<std::complex<double>>& roots);

    bool solve_for_y(std::vector<std::complex<double>>& roots, double y);

    typedef std::shared_ptr<PolynomialSolver> Ptr;

};


#endif //ALPHA_CONTROL_POLYNOMIAL_SOLVER_H
