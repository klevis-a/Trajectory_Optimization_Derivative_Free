//
// Created by klevis on 11/2/17.
//

#include <opt_problem.hpp>
#include <fstream>
#include <CostIneqComp.hpp>

using namespace opt_problems;

optProblem::optProblem(int dummy)
{
    std::cerr<<"optProblem dummy constructor is initialized, fix me!!!"<<std::endl;
}

optProblem::optProblem(TrajData data):_data(data)
{
    // Get optimization dimensions from robot kdl:
    m_dim=_data.kdl().dof;
    m_nec=0;
    m_nic = _data.kdl().dof * (_data.size()-1);// velocity constraints
}

double optProblem::objFunction(const vector<double> x) const
{
    CostIneqComp costComp(_data);
    double cost = costComp.computeCost(x);
    cout << "Cost : " << cost << endl;
    //costComp.writeThetaComputed();
    //costComp.writeXComputed();
    return cost;
}

vector<double> optProblem::objGradient(vector<double> x) const
{
    vector<double> retval(m_dim,0.0);
    return retval;
}

vector<double> optProblem::inEqConstraints(vector<double> x) const
{
    CostIneqComp ineq(_data);
    return ineq.computeIneq(x);
}

vector<double> optProblem::inEqGradient(vector<double> x) const
{
    vector<double> retval(m_dim*m_nic,0.0);
    return retval;
}

vector<double> optProblem::EqConstraints(vector<double> x) const
{
    // no constraint
    vector<double> ret(m_nec,0.0);
    return  ret;
}

vector<double> optProblem::eqGradient(vector<double> x) const
{
    vector<double> retval(m_dim*m_nec,0.0);
    return retval;
}

vector<vector<double>> optProblem::bounds() const
{
    vector<vector<double>> bound;
    vector<double> up_bounds(_data.kdl().up_bounds.begin(), _data.kdl().up_bounds.end());
    vector<double> low_bounds(_data.kdl().low_bounds.begin(), _data.kdl().low_bounds.end());

    bound.resize(2);
    bound[0]=low_bounds;
    bound[1]=up_bounds;
    return bound;

}

vector<double> optProblem::gradient(const vector<double> x) const
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
std::vector<std::pair<vector<double>::size_type, vector<double>::size_type>> optProblem::gradient_sparsity() const
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
vector<double> optProblem::sparse_gradient(const vector<double> x) const
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
