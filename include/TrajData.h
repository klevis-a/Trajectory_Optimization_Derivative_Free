#ifndef M20IA_PINV_OPT_TRAJDATA_H
#define M20IA_PINV_OPT_TRAJDATA_H

#include <vector>
#include <eigen3/Eigen/Dense>

typedef std::vector<Eigen::VectorXd>::size_type VecSz;
typedef std::vector<Eigen::MatrixXd>::size_type MatSz;
typedef std::vector<double>::size_type DblSz;

class TrajData
{
public:
    //constructor
    TrajData();
    TrajData(const std::string &workSpacePos, const Eigen::MatrixXd &tf);

    //read-only access members
    const double & dt(DblSz i) const;
    const Eigen::VectorXd & dx(VecSz i) const;
    const Eigen::MatrixXd & ct(MatSz i) const;
    const Eigen::MatrixXd & startingFrame() const;
    double size() const;

    //velocity limits
    static constexpr double expMultiplier=7.0;
    static constexpr double denominator=400.0;

private:
    //private member variables computed at init
    //time between steps
    std::vector<double> _dt;

    //dx
    std::vector<Eigen::VectorXd> _dx;

    //number of steps
    size_t _size;

    //holds transformation between each pose
    std::vector<Eigen::MatrixXd> _transforms;
    //holds starting pose
    Eigen::MatrixXd _startingFrame;
};

#endif //M20IA_PINV_OPT_TRAJDATA_H
