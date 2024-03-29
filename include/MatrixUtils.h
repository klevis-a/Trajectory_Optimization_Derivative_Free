#ifndef M20IA_PINV_OPT_MATRIXUTILS_H
#define M20IA_PINV_OPT_MATRIXUTILS_H

#include <eigen3/Eigen/Dense>

class MatrixUtils
{
public:
    static void pseudoinverse(const Eigen::MatrixXd &M, Eigen::MatrixXd &Minv, double tolerance);
    static Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd &M, double tolerance=1e-4);
    static Eigen::MatrixXd frameInverse(const Eigen::MatrixXd &input);
    static Eigen::VectorXd poseDiffIntrinsic(const Eigen::MatrixXd &start, const Eigen::MatrixXd &stop);
    static Eigen::VectorXd poseDiffIntrinsic(const Eigen::MatrixXd &diff);
    static Eigen::VectorXd poseDiffExtrinsic(const Eigen::MatrixXd &start, const Eigen::MatrixXd &stop);

private:
    MatrixUtils();
};

#endif //M20IA_PINV_OPT_MATRIXUTILS_H
