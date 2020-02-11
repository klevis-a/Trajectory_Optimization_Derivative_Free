//
// Created by klevis on 11/2/17.
//

#include <Robot_mocap_opt_problem.hpp>
#include <CostComp.hpp>
#include "MatrixUtils.hpp"

using namespace opt_problems;
using namespace std;
using namespace Eigen;

//this dummy constructor is needed for pagmo - it wants to be able to initialize empty problems
Robot_mocap_opt_problem::Robot_mocap_opt_problem(int dummy):_data(TrajData()), _kdl(RobotKDL()), _lowerCorner(vector<double>()), _upperCorner(vector<double>()), _velLimits(vector<double>())
{
    std::cerr<<"Robot_mocap_opt_problem dummy constructor is initialized, fix me!!!"<<std::endl;
}

Robot_mocap_opt_problem::Robot_mocap_opt_problem(const TrajData &data, const RobotKDL &kdl, const vector<double> &lowerCorner, const vector<double> &upperCorner, const vector<double> &velLimits):_data(data), _kdl(kdl), _lowerCorner(lowerCorner),_upperCorner(upperCorner),_velLimits(velLimits)
{
    // Get optimization dimensions from robot kdl:
    m_dim=_kdl.dof;
    //3 equality constraints:
    //1. overall trajectory position and orientation match
    //2. rotation about X for starting position is 0
    //3. rotation about Y for starting position is 0
    m_nec=3;
    //bounds on velocity (_kdl.dof * (_data.size()-1)
    //6 - bounds on initial starting position in workspace, 2 for each x,y,z
    //lower and upper bounds on joints: _kdl.dof * _data.size() *2
    m_nic = _kdl.dof * (_data.size()-1) + 6 + _kdl.dof * _data.size() *2;
}

double Robot_mocap_opt_problem::objFunction(const vector<double> &x) const
{
    CostComp costComp(_data, _kdl, _lowerCorner, _upperCorner, _velLimits);
    double cost = costComp.computeVelCost(x);
    cout << "Cost : " << cost << endl;
    return cost;
}

vector<double> Robot_mocap_opt_problem::inEqConstraints(const vector<double> &x) const
{
    CostComp ineq(_data, _kdl, _lowerCorner, _upperCorner, _velLimits);
    return ineq.computeIneq(x);
}

vector<double> Robot_mocap_opt_problem::EqConstraints(const vector<double> &x) const
{
    //the two vectors below indicate rotation about X and Y
    vector<double> ret(m_nec,0.0);
    VectorXd rotationAxisX(6);
    rotationAxisX << 0,0,0,1,0,0;
    VectorXd rotationAxisY(6);
    rotationAxisY << 0,0,0,0,1,0;

    //get the joint angles for the starting position
    vector<double> jointAngles0(_kdl.dof);
    for(int i=0; i<_kdl.dof;i++)
    {
        jointAngles0[i]=x[i];
    }

    //compute the forward kinematics
    MatrixXd startingFrameSugg = _kdl.getFKFrame(jointAngles0);
    VectorXd poseDiff = MatrixUtils::poseDiffExtrinsic(startingFrameSugg, _data.startingFrame());

    //no rotation about X or Y
    ret[0] = rotationAxisX.dot(poseDiff);
    ret[1] = rotationAxisY.dot(poseDiff);

    //make sure that the supplied trajectory matches our desired trajectory
    CostComp posComp(_data, _kdl, _lowerCorner, _upperCorner, _velLimits);
    ret[2] = posComp.computePosEquality(x);
    return  ret;
}

vector<double> Robot_mocap_opt_problem::objGradient(const vector<double> &x) const
{
    vector<double> retval(m_dim,0.0);
    return retval;
}

vector<double> Robot_mocap_opt_problem::inEqGradient(const vector<double> &x) const
{
    vector<double> retval(m_dim*m_nic,0.0);
    return retval;
}

vector<double> Robot_mocap_opt_problem::eqGradient(const vector<double> &x) const
{
    vector<double> retval(m_dim*m_nec,0.0);
    return retval;
}

vector<vector<double>> Robot_mocap_opt_problem::bounds() const
{
    vector<vector<double>> bound;
    vector<double> up_bounds(_kdl.up_bounds.begin(), _kdl.up_bounds.end());
    vector<double> low_bounds(_kdl.low_bounds.begin(), _kdl.low_bounds.end());

    bound.resize(2);
    bound[0]=low_bounds;
    bound[1]=up_bounds;
    return bound;

}

vector<double> Robot_mocap_opt_problem::gradient(const vector<double> &x) const
{
    vector<double> grad;
    vector<double> obj_grad=objGradient(x);
    vector<double> eq_grad=eqGradient(x);
    vector<double> ineq_grad=inEqGradient(x);

    grad.resize(m_dim+m_dim*m_nec+m_dim*m_nic,0.0);

    for(int i=0;i<m_dim;i++)
    {
        grad[i]=obj_grad[i];

    }

    for(int i=m_dim;i<m_dim+m_dim*m_nec;i++)
    {
        grad[i]=eq_grad[i-m_dim];
    }


    for(int i=m_dim+m_dim*m_nec;i<m_dim+m_dim*m_nec+m_dim*m_nic;i++)
    {

        grad[i]=ineq_grad[i-m_dim-m_dim*m_nec];
    }

    return grad;
}

#if SPARSE_GRADIENT==true
std::vector<std::pair<vector<double>::size_type, vector<double>::size_type>> Robot_mocap_opt_problem::gradient_sparsity() const
{
  std::vector<std::pair<vector<double>::size_type, vector<double>::size_type>> retval;
  // Objective gradient is dense
  for (int i=0;i<m_dim;++i)
  {
    retval.emplace_back(0,i);
  }
  // Equality constraints:
  for (int i=0;i<m_nec;++i)
  {
    retval.emplace_back(i+1,i);
  }
  // inequality constraints:
  for (int i=0;i<m_nic;++i)
  {
    retval.emplace_back(i+m_nec+1,i);
    retval.emplace_back(i+m_nec+1,i+robot_kdl_.dof);

  }
  return retval;
}
vector<double> Robot_mocap_opt_problem::sparse_gradient(const vector<double> x) const
{
  vector<double> grad;
  vector<double> obj_grad=objGradient(x);
  vector<double> eq_grad=eqGradient(x);
  vector<double> ineq_grad=inEqGradient(x);

  grad.resize(m_dim,0.0);

  for(int i=0;i<m_dim;i++)
  {
    grad[i]=obj_grad[i];
  }

  for(int i=m_dim;i<m_dim+m_dim*m_nec;i++)
  {
    if(eq_grad[i-m_dim]!=0.0)
    {
      grad.emplace_back(eq_grad[i-m_dim]);
    }
  }


  for(int i=m_dim+m_dim*m_nec;i<m_dim+m_dim*m_nec+m_dim*m_nic;i++)
  {
    if(ineq_grad[i-m_dim-m_dim*m_nec]!=0.0)
    {
      grad.emplace_back(ineq_grad[i-m_dim-m_dim*m_nec]);
    }
  }

  return grad;
}

#endif
