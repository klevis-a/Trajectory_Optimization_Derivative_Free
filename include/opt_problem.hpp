#ifndef OPT_PROBLEM_HPP
#define OPT_PROBLEM_HPP
#ifndef SPARSE_GRADIENT
#define SPARSE_GRADIENT false
#endif
#include <iostream>


#include <TrajData.hpp>

// Eigen library for vectors and matrices:
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>

using namespace std;
namespace opt_problems
{
  class optProblem
  {
  public:
    // constructor
    optProblem(int dummy);// dummy since pagmo requires default values for variables
    optProblem(TrajData data);

    // functions
    double objFunction(vector<double> x) const; 
    vector<double> inEqConstraints(vector<double> x) const;
    vector<double> EqConstraints(vector<double> x) const;
    vector<double> gradient(vector<double> x) const;
    vector<double> objGradient(vector<double> x) const;
    vector<double> inEqGradient(vector<double> x) const;
    vector<double> eqGradient(vector<double> x) const;
    
    vector<vector<double>> bounds() const;
    #if SPARSE_GRADIENT==true
    vector<double> sparse_gradient(vector<double> x) const;
    std::vector<std::pair<vector<double>::size_type, vector<double>::size_type>> gradient_sparsity() const;
    #endif
    int m_dim,m_nec,m_nic;
  private:
    TrajData _data;
  };
}
#endif
