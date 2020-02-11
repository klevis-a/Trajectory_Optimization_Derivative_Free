#include <TrajData.hpp>
#include <csv.h>
#include "MatrixUtils.hpp"

constexpr double TrajData::expMultiplier;
constexpr double TrajData::denominator;

using namespace std;
using namespace Eigen;

//here empty constructor is needed so problem can be initialized empty - see mocap_traj_problem.cpp
TrajData::TrajData()
{
}

TrajData::TrajData(const string &workSpacePos, const MatrixXd &tf)
{
    //read in the trajectory
    io::CSVReader<17> input(workSpacePos);
    double tb,t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33;

    size_t rowNum = 0;
    MatrixXd firstFrameInv;
    MatrixXd tfInv = MatrixUtils::frameInverse(tf);

    while(input.read_row(tb,t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33))
    {
        MatrixXd currentFrame(4,4);
        currentFrame << t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33;
        if(rowNum>0)
        {
            _dt.push_back(tb);
            MatrixXd currentTransform = firstFrameInv*currentFrame*tfInv;
            _transforms.push_back(currentTransform);
            _dx.push_back(MatrixUtils::poseDiffIntrinsic(currentTransform));
        }
        else
        {
            _startingFrame = currentFrame*tfInv;
            firstFrameInv = MatrixUtils::frameInverse(_startingFrame);
        }
        rowNum++;
    }
    _size = rowNum;
}

const VectorXd & TrajData::dx(VecSz i) const
{
    return _dx[i];
}

const double & TrajData::dt(DblSz i) const
{
    return _dt[i];
}

double TrajData::size() const
{
    return _size;
}

const MatrixXd & TrajData::ct(MatSz i) const
{
    return _transforms[i];
}

const MatrixXd & TrajData::startingFrame() const
{
    return _startingFrame;
}