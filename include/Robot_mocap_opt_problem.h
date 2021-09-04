#ifndef M20IA_PINV_OPT_ROBOT_MOCAP_OPT_PROBLEM_H
#define M20IA_PINV_OPT_ROBOT_MOCAP_OPT_PROBLEM_H

#ifndef SPARSE_GRADIENT
#define SPARSE_GRADIENT false
#endif

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "TrajData.h"
#include "RobotKDL.h"

class Robot_mocap_opt_problem
{
public:
    // constructor
    Robot_mocap_opt_problem(int dummy); // dummy since pagmo requires default values for variables
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
#endif //M20IA_PINV_OPT_ROBOT_MOCAP_OPT_PROBLEM_H
