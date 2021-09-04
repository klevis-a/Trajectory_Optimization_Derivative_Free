#ifndef M20IA_PINV_OPT_COSTCOMP_H
#define M20IA_PINV_OPT_COSTCOMP_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include "TrajData.h"
#include "RobotKDL.h"

class CostComp
{
public:
    //constructor
    CostComp(const TrajData &data, const RobotKDL &kdl, const std::vector<double> &lowerCorner, const std::vector<double> &upperCorner, const std::vector<double> &velLimits);

    //cost functions
    void computeDerived(const std::vector<double> &initial_joints);
    double computePosEquality(const std::vector<double> &x);
    std::vector<double> computeIneq(const std::vector<double> &x);
    double computeVelCost(const std::vector<double> &x);

    //accessor functions
    const Eigen::VectorXd & dtheta(VecSz i) const;
    const Eigen::VectorXd & dxAchieved(VecSz i) const;
    const Eigen::MatrixXd & xAchieved(VecSz i) const;
    const Eigen::VectorXd & theta(VecSz i) const;

    //output methods
    void writeXAchieved(const std::string file) const;
    void writeTheta(const std::string file) const;

private:
    const TrajData &_data;
    const RobotKDL &_kdl;
    const std::vector<double> &_lowerCorner;
    const std::vector<double> &_upperCorner;
    const std::vector<double> &_velLimits;

    //private members variables recomputed with new joint angles
    //computed x position for each step
    std::vector<Eigen::MatrixXd> _xComputed;
    //computed dx for each time step
    std::vector<Eigen::VectorXd> _dxAchieved;
    //computed x position for each step
    std::vector<Eigen::MatrixXd> _xAchieved;
    //dtheta at each timestep
    std::vector<Eigen::VectorXd> _dtheta;
    //theta at each timestep
    std::vector<Eigen::VectorXd> _theta;

    //helper functions
    void clearAll();
};

#endif //M20IA_PINV_OPT_COSTCOMP_H
