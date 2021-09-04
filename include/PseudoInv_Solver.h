//
// Created by klevis on 1/12/18.
//

#ifndef M20IA_PINV_OPT_PSEUDOINV_SOLVER_H
#define M20IA_PINV_OPT_PSEUDOINV_SOLVER_H

#include <string>
#include <pagmo/pagmo.hpp>
#include "TrajData.h"
#include "ConfigParser.h"
#include "RobotKDL.h"

class PseudoInv_Solver {
public:
    PseudoInv_Solver(const ConfigParser &configParser, const std::string &dataFile, const std::vector<std::vector<double>> &seeds, const Eigen::MatrixXd &tf);
    void solve();
    void printResults(const std::string &jointsFile) const;
private:
    pagmo::algorithm _algo;
    pagmo::population _pop;
    pagmo::population _pop_result;
    TrajData _data;
    RobotKDL _robot;
    const ConfigParser &_configParser;
};

#endif //M20IA_PINV_OPT_PSEUDOINV_SOLVER_H
