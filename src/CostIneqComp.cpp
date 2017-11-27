#include <CostIneqComp.hpp>
#include <PseudoUtils.hpp>
#include <fstream>

CostIneqComp::CostIneqComp(const TrajData &data):_data(data)
{}

void CostIneqComp::writeXComputed(const string workSpacePos) const
{
    ofstream csvFile;
    csvFile.open(workSpacePos);

    for(vector<VectorXd>::const_iterator iter = xCompVec().begin(); iter!=xCompVec().end(); ++iter)
    {
        csvFile << (*iter)[0] << "," << (*iter)[1] << "," << (*iter)[2] << "," << (*iter)[3] << "," << (*iter)[4] << "," << (*iter)[5] << endl;
    }

    csvFile.close();
}

void CostIneqComp::writeThetaComputed(const string workSpacePos) const
{
    ofstream csvFile;
    csvFile.open(workSpacePos);

    for(vector<vector<double>>::const_iterator iter = _theta.begin(); iter!=_theta.end(); ++iter)
    {
        csvFile << (*iter)[0] << "," << (*iter)[1] << "," << (*iter)[2] << "," << (*iter)[3] << "," << (*iter)[4] << "," << (*iter)[5] << endl;
    }

    csvFile.close();
}

double CostIneqComp::velocityCost(const VectorXd &jointVelocity)
{
    VectorXd velPercentage = jointVelocity.cwiseAbs().cwiseQuotient(TrajData::vel_limits);
    //cout << "Velocity Percentage: " << velPercentage.transpose() << endl;
    VectorXd velPercentageScaled = velPercentage*TrajData::expMultiplier;
    VectorXd costPerJoint = velPercentageScaled.unaryExpr([](const double &elem) {return std::exp(elem);}).unaryExpr([](const double &elem){return elem-1;})/TrajData::denominator;
    return costPerJoint.sum();
}

double CostIneqComp::positionCost(const Eigen::VectorXd &suggestedPosition, const Eigen::VectorXd &desiredPosition)
{
    VectorXd diff = _data.posDiff(suggestedPosition, desiredPosition);
    //cout << "Unscaled Diff" << diff.transpose() << endl;
    //scale the orientation
    //diff[3]*=TrajData::orientationScale;
    //diff[4]*=TrajData::orientationScale;
    //diff[5]*=TrajData::orientationScale;
    //cout << "Scaled Diff" << diff.transpose() << endl;
    return diff.dot(diff);
}

double CostIneqComp::computeCost(const vector<double> &initial_joints)
{
    //cout << "Computed velocity " << endl;
    computeDerived(initial_joints);

    //compute position costs
    double totalPositionCost=0.0;
    vector<double> posVec(_data.size()-1);
    for(size_t i=0; i!=_data.size()-1;i++)
    {
        VectorXd diff = _data.dx(i)-dxComputed(i);
        double currentPos = diff.dot(diff);
        posVec[i]=currentPos;
        totalPositionCost += currentPos;
    }

    return totalPositionCost;
}

std::vector<double> CostIneqComp::computeIneq(const std::vector<double> &x)
{
    computeDerived(x);
    vector<double> ret((_data.size()-1)*_data.kdl().dof);

    for(size_t i=0; i<_data.size()-1;i++)
    {
        for(size_t j=0; j<_data.kdl().dof; j++)
        {
            ret[i*_data.kdl().dof+j]=abs(dtheta(i)[j]/_data.dt(i))-_data.vel_limits[j];
        }
    }

    return ret;
}

void CostIneqComp::clearAll()
{
    _dxComputed.clear();
    _xComputed.clear();
    _dtheta.clear();
    _theta.clear();
    _J.clear();
    _JPinv.clear();
}
void CostIneqComp::computeDerived(const vector<double> &initial_joints)
{
    //clear all computed member variables
    clearAll();

    //push the initial joint angles
    _theta.push_back(initial_joints);
    //cout << 0 << " " << Map<VectorXd>(_theta[0].data(), _theta[0].size()).transpose() << endl;

    //compute the forward kinematics and store them
    vector<double> currentCartPos = _data.kdl().getFK(0,initial_joints, true);
    VectorXd currentCartPosXd = Map<VectorXd>(currentCartPos.data(), currentCartPos.size());
    _xComputed.push_back(currentCartPosXd);
    MatrixXd firstPosE = _data.kdl().pose_to_frame(currentCartPos);
    cout << "Total Frame: " << endl;
    cout << firstPosE << endl;
    MatrixXd firstPosR = firstPosE.topLeftCorner<3,3>();
    cout << "Just Rotation: " << endl;
    cout << firstPosR << endl;
    VectorXd firstPosT = firstPosE.col(3).head<3>();
    cout << "Just Translation: " << endl;
    cout << firstPosT << endl;
    //cout << _xComputed[0].transpose() << " " << endl;

    for(vector<double>::size_type i=0; i!=_data.size()-1; i++)
    {
        //current joint
        vector<double> jointAngle = theta(i);
        //compute Jacobian at current joint
        _J.push_back(_data.kdl().getJacobian(0, jointAngle));
        //compute Jacobian Pseudoinverse at current joint
        _JPinv.push_back(PseudoUtils::pseudoinverse(_J[i]));
        //delta theta is Jacobian Pinv times dx
        //VectorXd dxCurrent = _data.cdx(i)-_data.posDiff(_xComputed.front(), _xComputed[i]);
        //VectorXd dxCurrent = _data.dx(i);
        //VectorXd dxCurrent = _data.posDiff(_xComputed.back(),_xComputed.front()+_data.cdx(i));
        MatrixXd cfE = diffFrameCompute(firstPosR, _data.rotDiff(i), firstPosT, _data.transDiff(i));
        vector<double> cv = _data.kdl().frame_to_pose_(cfE,true);
        VectorXd cE = Map<VectorXd>(cv.data(), cv.size());
        VectorXd dxCurrent = _data.posDiff(_xComputed.back(), cE);
        _dtheta.push_back(JPinv(i) * dxCurrent);
        //compute theta now
        VectorXd thetaXd = Map<VectorXd>(jointAngle.data(), jointAngle.size())+dtheta(i);
        vector<double> thetaC(thetaXd.data(), thetaXd.data()+thetaXd.size());
        _theta.push_back(thetaC);
        //cout << i+1 << " " << Map<VectorXd>(_theta[i+1].data(), _theta[i+1].size()).transpose() << endl;
        //from the new theta compute the forward kinematics
        vector<double> xNew=_data.kdl().getFK(0,thetaC, true);
        VectorXd xNewXd = Map<VectorXd>(xNew.data(), xNew.size());
        //simply add it so we can keep track of the difference
        _dxComputed.push_back(_data.posDiff(xComputed(i), xNewXd));
        //push back the new computed x
        _xComputed.push_back(xNewXd);
        //cout << _xComputed[i+1].transpose() << endl;
    }
}

MatrixXd CostIneqComp::diffFrameCompute(const Eigen::MatrixXd startO, const Eigen::MatrixXd diffO, const Eigen::VectorXd startP, const Eigen::VectorXd diffP)
{
    cout << "Start Rotation" << endl;
    cout << startO << endl;
    cout << "Diff Rotation" << endl;
    cout << diffO << endl;
    cout << "Start Position" << endl;
    cout << startP << endl;
    cout << "Diff Position" << endl;
    cout << diffP << endl;
    Eigen::MatrixXd frame(4,4);
    frame.setIdentity();
    MatrixXd oComp = diffO*startO;
    frame.topLeftCorner<3,3>()=oComp;
    VectorXd tComp = startP+diffP;
    frame.col(3).head<3>() = tComp;
    cout << "Created Frame" << endl;
    cout << frame << endl;
    return frame;
}

const MatrixXd & CostIneqComp::J(MatSz i) const
{
    return _J[i];
}

const MatrixXd & CostIneqComp::JPinv(MatSz i) const
{
    return _JPinv[i];
}

const VectorXd & CostIneqComp::dtheta(VecSz i) const
{
    return _dtheta[i];
}

const VectorXd & CostIneqComp::dxComputed(VecSz i) const
{
    return _dxComputed[i];
}
const VectorXd & CostIneqComp::xComputed(VecSz i) const
{
    return _xComputed[i];
}
const vector<double> & CostIneqComp::theta(VecSz i) const
{
    return _theta[i];
}

const vector<VectorXd> & CostIneqComp::xCompVec() const
{
    return _xComputed;
}