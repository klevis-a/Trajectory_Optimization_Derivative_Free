#include <PseudoUtils.hpp>

void PseudoUtils::pseudoinverse(const Eigen::MatrixXd &M, Eigen::MatrixXd &Minv, double tolerance)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svdOfM(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::MatrixXd U = svdOfM.matrixU();
    const Eigen::MatrixXd V = svdOfM.matrixV();
    const Eigen::VectorXd S = svdOfM.singularValues();

    Eigen::VectorXd Sinv = S;
    double maxsv = 0;
    for (std::size_t i = 0; i < S.rows(); ++i)
        if (fabs(S(i)) > maxsv)
            maxsv = fabs(S(i));
    for (std::size_t i = 0; i < S.rows(); ++i)
    {
        //Those singular values smaller than a percentage of the maximum singular value are removed
        if (fabs(S(i)) > maxsv * tolerance)
            Sinv(i) = 1.0 / S(i);
        else
            Sinv(i) = 0;
    }

    Minv = V * Sinv.asDiagonal() * U.transpose();
}

Eigen::MatrixXd PseudoUtils::pseudoinverse(const Eigen::MatrixXd &M, double tolerance)
{
    Eigen::MatrixXd Minv;
    pseudoinverse(M, Minv, tolerance);
    return Minv;
}