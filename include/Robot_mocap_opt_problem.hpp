#ifndef OPT_PROBLEM_HPP
#define OPT_PROBLEM_HPP
#ifndef SPARSE_GRADIENT
#define SPARSE_GRADIENT false
#endif
#include <iostream>
#include <TrajData.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <RobotKDL.h>
#include <vector>

namespace opt_problems
{
    class Robot_mocap_opt_problem
    {
    public:
        // constructor
        Robot_mocap_opt_problem(int dummy);// dummy since pagmo requires default values for variables
        Robot_mocap_opt_problem(const TrajData &data, const RobotKDL &kdl, const std::vector<double> &lowerCorner, const std::vector<double> &upperCorner, const std::vector<double> &velLimits);

        // functions
        double objFunction(const std::vector<double> &x) const;
        std::vector<double> inEqConstraints(const std::vector<double> &x) const;
        std::vector<double> EqConstraints(const std::vector<double> &x) const;
        std::vector<double> gradient(const std::vector<double> &x) const;
        std::vector<double> objGradient(const std::vector<double> &x) const;
        std::vector<double> inEqGradient(const std::vector<double> &x) const;
        std::vector<double> eqGradient(const std::vector<double> &x) const;

        std::vector<std::vector<double>> bounds() const;
#if SPARSE_GRADIENT==true
        vector<double> sparse_gradient(vector<double> x) const;
        std::vector<std::pair<vector<double>::size_type, vector<double>::size_type>> gradient_sparsity() const;
#endif
        int m_dim,m_nec,m_nic;
    private:
        const TrajData & _data;
        const RobotKDL & _kdl;
        const std::vector<double> &_lowerCorner;
        const std::vector<double> &_upperCorner;
        const std::vector<double> &_velLimits;
    };
}
#endif
