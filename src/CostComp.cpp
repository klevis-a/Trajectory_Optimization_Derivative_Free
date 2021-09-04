#include <fstream>

#include "CostComp.h"
#include "MatrixUtils.h"

using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Map;

CostComp::CostComp(const TrajData &data, const RobotKDL &kdl, const vector<double> &lowerCorner, const vector<double> &upperCorner, const vector<double> &velLimits):
                    _data(data),_kdl(kdl),_lowerCorner(lowerCorner),_upperCorner(upperCorner),_velLimits(velLimits) {}

void CostComp::writeXAchieved(const std::string workSpacePos) const
{
    std::ofstream csvFile;
    csvFile.open(workSpacePos);

    for(auto iter = _xAchieved.begin(); iter!=_xAchieved.end(); ++iter)
    {
        csvFile << (*iter)(0,0) << "," << (*iter)(0,1) << "," << (*iter)(0,2) << "," << (*iter)(0,3)
                << (*iter)(1,0) << "," << (*iter)(1,1) << "," << (*iter)(1,2) << "," << (*iter)(1,3)
                << (*iter)(2,0) << "," << (*iter)(2,1) << "," << (*iter)(2,2) << "," << (*iter)(2,3)
                << (*iter)(3,0) << "," << (*iter)(3,1) << "," << (*iter)(3,2) << "," << (*iter)(3,3)
                << std::endl;
    }

    csvFile.close();
}

void CostComp::writeTheta(const std::string workSpacePos) const
{
    std::ofstream csvFile;
    csvFile.open(workSpacePos);

    for(auto iter = _theta.begin(); iter!=_theta.end(); ++iter)
    {
        csvFile << (*iter)[0] << "," << (*iter)[1] << "," << (*iter)[2] << "," << (*iter)[3] << "," << (*iter)[4] << "," << (*iter)[5] << std::endl;
    }

    csvFile.close();
}

double CostComp::computeVelCost(const vector<double> &x)
{
    computeDerived(x);
    vector<double> vel_cost(_data.size()-1,0.0);
    const VectorXd velocityLimits = Map<const VectorXd>(_velLimits.data(), _velLimits.size());

    for(size_t i=0; i<_data.size()-1; i++)
    {
        VectorXd jointVelocity = dtheta(i)/_data.dt(i);
        VectorXd velPercentage = jointVelocity.cwiseAbs().cwiseQuotient(velocityLimits);
        VectorXd costPerJoint = velPercentage.unaryExpr([](const double &elem) {
            double retVal=0.0;
            static double expEndVal = (std::exp(TrajData::expMultiplier*1.2)-1)/TrajData::denominator;
            static double m = (100-expEndVal)/(2-1.2);
            if(elem<=1.2)
            {
                retVal=(std::exp(TrajData::expMultiplier*elem)-1)/TrajData::denominator;
            } else
            {
                retVal=m*(elem-1.2)+expEndVal;
            }
            return retVal;});
        vel_cost[i]=costPerJoint.sum();
    }

    return std::accumulate(vel_cost.begin(), vel_cost.end(), 0.0);
}

double CostComp::computePosEquality(const vector<double> &initial_joints)
{
    computeDerived(initial_joints);

    //compute position costs
    double totalPositionCost=0.0;
    vector<double> posVec(_data.size()-1);
    for(size_t i=0; i!=_data.size()-1;i++)
    {
        VectorXd diff = _data.dx(i)-dxAchieved(i);
        double currentPos = diff.dot(diff);
        posVec[i]=currentPos;
        totalPositionCost += currentPos;
    }

    return totalPositionCost;
}

vector<double> CostComp::computeIneq(const vector<double> &x)
{
    computeDerived(x);

    //set boundaries on velocity
    vector<double> retVel((_data.size()-1)*_kdl.dof);

    for(size_t i=0; i<_data.size()-1;i++)
    {
        for(size_t j=0; j<_kdl.dof; j++)
        {
            retVel[i*_kdl.dof+j]=abs(dtheta(i)[j]/_data.dt(i))-_velLimits[j];
        }
    }

    //int count = std::count_if(retVel.begin(), retVel.end(), [](int n) {return n >= 0.001;} );
    //std::cout << "Violated velocity count: " << count << std::endl;

    //set joint limits - note that the bounds on the problems only apply to the initial position
    //therefore inequality constraints on the rest of the positions are necessary
    const vector<double> &ub = _kdl.up_bounds;
    const vector<double> &lb = _kdl.low_bounds;

    for(int i=0;i<_data.size(); i++) {
        for (int j = 0; j < 6; j++) {
            retVel.push_back(theta(i)[j] - ub[j]);
            retVel.push_back(lb[j] - theta(i)[j]);
        }
    }

    //count = std::count_if(retVel.begin(), retVel.end(), [](int n) {return n >= 0.001;} );
    //std::cout << "Violated joint count: " << count << std::endl;

    //this places workspace bounds on the starting position
    VectorXd initPosition = _xAchieved[0].topRightCorner<3,1>();
    //x bounds
    retVel.push_back(initPosition[0]-_upperCorner[0]);
    retVel.push_back(_lowerCorner[0]-initPosition[0]);
    //y bounds
    retVel.push_back(initPosition[1]-_upperCorner[1]);
    retVel.push_back(_lowerCorner[1]-initPosition[1]);
    //z bounds
    retVel.push_back(initPosition[2]-_upperCorner[2]);
    retVel.push_back(_lowerCorner[2]-initPosition[2]);
    return retVel;
}

void CostComp::clearAll()
{
    _xComputed.clear();
    _xAchieved.clear();
    _dxAchieved.clear();
    _dtheta.clear();
    _theta.clear();
}
void CostComp::computeDerived(const vector<double> &initial_joints)
{
    //clear all computed member variables
    clearAll();

    //push the initial joint angles
    const VectorXd initJoints = Map<const VectorXd>(initial_joints.data(), initial_joints.size());
    _theta.push_back(initJoints);

    MatrixXd firstFrame = _kdl.getFKFrame(initJoints);
    MatrixXd firstFrameInv = MatrixUtils::frameInverse(firstFrame);
    _xComputed.push_back(firstFrame);
    _xAchieved.push_back(firstFrame);

    for(vector<double>::size_type i=0; i!=_data.size()-1; i++)
    {
        //current joint
        VectorXd thetaCurrent = _theta.back();
        //compute Jacobian at current joint
        MatrixXd J = _kdl.getJacobian(thetaCurrent);
        //compute Jacobian Pseudoinverse at current joint
        MatrixXd JPinv = MatrixUtils::pseudoinverse(J);

        //compute current frame based on transformations
        MatrixXd currentDesFrame = firstFrame*_data.ct(i);
        _xComputed.push_back(currentDesFrame);

        //compute the difference between the previously achieved position
        //and the current desired pose
        //note the difference needs to be extrinsic because the Jacobian (therefore the pseudoinverse)
        //are in the world frame
        VectorXd currentDx = MatrixUtils::poseDiffExtrinsic(_xAchieved.back(), currentDesFrame);
        //calculate dtheta from that
        VectorXd dtheta = JPinv * currentDx;
        _dtheta.push_back(dtheta);

        //compute theta now
        VectorXd thetaNew = thetaCurrent + dtheta;
        _theta.push_back(thetaNew);

        //from the new theta compute the forward kinematics
        MatrixXd xNew = _kdl.getFKFrame(thetaNew);
        _dxAchieved.push_back(MatrixUtils::poseDiffIntrinsic(firstFrameInv * xNew));
        _xAchieved.push_back(xNew);
    }
}

const VectorXd& CostComp::dtheta(VecSz i) const
{
    return _dtheta[i];
}

const VectorXd& CostComp::dxAchieved(VecSz i) const
{
    return _dxAchieved[i];
}

const MatrixXd& CostComp::xAchieved(VecSz i) const
{
    return _xAchieved[i];
}
const VectorXd& CostComp::theta(VecSz i) const
{
    return _theta[i];
}
