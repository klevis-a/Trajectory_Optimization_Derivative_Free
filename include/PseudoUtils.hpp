#include <eigen3/Eigen/Dense>

class PseudoUtils
{
    public:
        static void pseudoinverse(const Eigen::MatrixXd &M, Eigen::MatrixXd &Minv, double tolerance);
        static Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd &M, double tolerance=1e-4);
    private:
        PseudoUtils();
};