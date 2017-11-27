#include <TrajData.hpp>
#include <csv.h>
#include <fstream>
#include <cmath>

constexpr double TrajData::expMultiplier;
constexpr double TrajData::denominator;
constexpr double TrajData::orientationScale;
Eigen::VectorXd TrajData::vel_limits=[]{
    Eigen::VectorXd t(6);
    t << 3.403392,3.054326,3.141593,6.283185,6.283185,9.599311;
    return t;}();

using namespace std;
using namespace Eigen;

TrajData::TrajData()
{
}

Eigen::MatrixXd TrajData::frameInverse(const vector<double> &pose) const
{
    //compute inverse of sawbone transform
    tf::Matrix3x3 R;
    R.setRPY(pose[3],pose[4],pose[5]);
    tf::Matrix3x3 Rinv = R.inverse();
    Eigen::MatrixXd RInvE(3,3);
    for (int i=0;i<3;i++)
    {
        RInvE(i,0)=Rinv[i].x();
        RInvE(i,1)=Rinv[i].y();
        RInvE(i,2)=Rinv[i].z();
    }
    Eigen::VectorXd trans(3);
    trans << pose[0],pose[1],pose[2];
    Eigen::VectorXd transI = -RInvE * trans;
    Eigen::MatrixXd inverse(4,4);
    inverse.setIdentity();
    for (int i=0;i<3;i++)
    {
        inverse(i,0)=RInvE(i,0);
        inverse(i,1)=RInvE(i,1);
        inverse(i,2)=RInvE(i,2);
    }
    inverse(0,3)=transI(0);
    inverse(1,3)=transI(1);
    inverse(2,3)=transI(2);

    return inverse;
}

Eigen::MatrixXd TrajData::rotationDiff(const MatrixXd &initPose, const MatrixXd &finalPose) const
{
    MatrixXd initRot = initPose.topLeftCorner<3,3>();
    MatrixXd finalRot = finalPose.topLeftCorner<3,3>();
    return finalRot*initRot.transpose();
}

Eigen::VectorXd TrajData::transDiff(const MatrixXd &initPose, const MatrixXd &finalPose) const
{
    VectorXd initTrans = initPose.col(3).head<3>();
    VectorXd finalTrans = finalPose.col(3).head<3>();
    return finalTrans-initTrans;
}

TrajData::TrajData(const string &workSpacePos, const manipulator_kdl::robotKDL kdl):_kdl(kdl)
{
    //read in the trajectory
    io::CSVReader<7> input(workSpacePos);
    double tb,px,py,pz,yaw,pitch,roll;

    size_t rowNum = 0;
    VectorXd previousRow;
    MatrixXd firstFrameIn;
    MatrixXd firstFrame;

    while(input.read_row(tb,px,py,pz,yaw,pitch,roll))
    {
        vector<double> crow{px,py,pz,yaw,pitch,roll};
        VectorXd currentRow = Map<VectorXd>(crow.data(), crow.size());
        if(rowNum>0)
        {
            _dx.push_back(posDiff(previousRow, currentRow));
            //_cdx.push_back(posDiff(firstRow, currentRow));
            _dt.push_back(tb);
            MatrixXd cf = kdl.pose_to_frame(crow);
            //_transforms.push_back(cf*firstFrameIn);
            //cout << "Transform " << rowNum << endl;
            //cout << _transforms.back() << endl;
            _rotDiff.push_back(rotationDiff(firstFrame, cf));
            _transDiff.push_back(transDiff(firstFrame, cf));
        }
        else
        {
            firstFrame = _kdl.getFKEigen(0, crow);
            //firstFrameIn = frameInverse(crow);
        }
        rowNum++;
        previousRow = currentRow;
    }
    _size = rowNum;
}

VectorXd TrajData::posDiff(const VectorXd &start,const VectorXd &stop) const
{
	VectorXd diff = stop-start;
	vector<double> startO = {start[3], start[4], start[5]};
	vector<double> stopO = {stop[3], stop[4], stop[5]};
	vector<double> orient_err=_kdl.euler_diff(startO,stopO);
	diff[3] = orient_err[0];
	diff[4] = orient_err[1];
	diff[5] = orient_err[2];
	return diff;
}

const VectorXd & TrajData::dx(VecSz i) const
{
    return _dx[i];
}

// const VectorXd & TrajData::cdx(VecSz i) const
// {
//     return _cdx[i];
// }

const double & TrajData::dt(DblSz i) const
{
    return _dt[i];
}

double TrajData::size() const
{
    return _size;
}

const manipulator_kdl::robotKDL & TrajData::kdl() const
{
    return _kdl;
}

// const Eigen::MatrixXd & TrajData::ct(MatSz i) const
// {
//     return _transforms[i];
// }

const Eigen::MatrixXd & TrajData::rotDiff(MatSz i) const
{
    return _rotDiff[i];
}

const Eigen::VectorXd & TrajData::transDiff(VecSz i) const
{
    return _transDiff[i];
}