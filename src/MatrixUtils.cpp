#include <TrajData.hpp>
#include <MatrixUtils.hpp>

using namespace std;
using namespace Eigen;

void MatrixUtils::pseudoinverse(const MatrixXd &M, MatrixXd &Minv, double tolerance)
{
    JacobiSVD<Eigen::MatrixXd> svdOfM(M, ComputeThinU | ComputeThinV);
    const MatrixXd U = svdOfM.matrixU();
    const MatrixXd V = svdOfM.matrixV();
    const VectorXd S = svdOfM.singularValues();

    VectorXd Sinv = S;
    double maxsv = 0;
    for (size_t i = 0; i < S.rows(); ++i)
        if (fabs(S(i)) > maxsv)
            maxsv = fabs(S(i));
    for (size_t i = 0; i < S.rows(); ++i)
    {
        //Those singular values smaller than a percentage of the maximum singular value are removed
        if (fabs(S(i)) > maxsv * tolerance)
            Sinv(i) = 1.0 / S(i);
        else
            Sinv(i) = 0;
    }

    Minv = V * Sinv.asDiagonal() * U.transpose();
}

MatrixXd MatrixUtils::pseudoinverse(const MatrixXd &M, double tolerance)
{
    MatrixXd Minv;
    pseudoinverse(M, Minv, tolerance);
    return Minv;
}

MatrixXd MatrixUtils::frameInverse(const MatrixXd &input)
{
    MatrixXd inverse(4, 4);
    inverse.setIdentity();
    inverse.topLeftCorner<3,3>()=input.topLeftCorner<3,3>().transpose();
    inverse.topRightCorner<3,1>()=-1*input.topLeftCorner<3,3>().transpose()*input.topRightCorner<3,1>();
    return inverse;
}

VectorXd MatrixUtils::poseDiffIntrinsic(const MatrixXd &start, const MatrixXd &stop)
{
    MatrixXd diff = MatrixUtils::frameInverse(start) * stop;
    AngleAxisd ax(diff.topLeftCorner<3,3>());
    VectorXd axScaled = ax.axis() * ax.angle();
    VectorXd pos = diff.topRightCorner<3,1>();
    VectorXd totalDiff(axScaled.size() + pos.size());
    totalDiff << pos,axScaled;
    return totalDiff;
}

VectorXd MatrixUtils::poseDiffIntrinsic(const MatrixXd &diff)
{
    AngleAxisd ax(diff.topLeftCorner<3,3>());
    VectorXd axScaled = ax.axis() * ax.angle();
    VectorXd pos = diff.topRightCorner<3,1>();
    VectorXd totalDiff(axScaled.size() + pos.size());
    totalDiff << pos,axScaled;
    return totalDiff;
}

VectorXd MatrixUtils::poseDiffExtrinsic(const MatrixXd &start, const MatrixXd &stop)
{
    MatrixXd diff = stop * MatrixUtils::frameInverse(start);
    AngleAxisd ax(diff.topLeftCorner<3,3>());
    VectorXd axScaled = ax.axis() * ax.angle();
    VectorXd pos = stop.topRightCorner<3,1>() - start.topRightCorner<3,1>();
    VectorXd totalDiff(axScaled.size() + pos.size());
    totalDiff << pos,axScaled;
    return totalDiff;
}