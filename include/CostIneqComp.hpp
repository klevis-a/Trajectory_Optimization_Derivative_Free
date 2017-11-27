#ifndef COSTINEQCOMP_H
#define COSTINEQCOMP_H
#include <eigen3/Eigen/Dense>
#include <vector>
#include <TrajData.hpp>

class CostIneqComp
{
    public:
        //constructor
        CostIneqComp(const TrajData &data);
        //cost functions
        double computeCost(const std::vector<double> &x);
        std::vector<double> computeIneq(const std::vector<double> &x);

        //accessor functions
		const Eigen::MatrixXd & J(MatSz i) const;
		const Eigen::MatrixXd & JPinv(MatSz i) const;
		const Eigen::VectorXd & dtheta(VecSz i) const;
		const Eigen::VectorXd & dxComputed(VecSz i) const;
		const Eigen::VectorXd & xComputed(VecSz i) const;
		const vector<double> & theta(VecSz i) const;
		const vector<Eigen::VectorXd> & xCompVec() const;

		//output methods
		void writeXComputed(const std::string file="/media/sf_VMShare/newOutput/computedCart.csv") const;
		void writeThetaComputed(const std::string file="/media/sf_VMShare/newOutput/computedJoints.csv") const;

    private:
        const TrajData &_data;
        //private members variables recomputed with new joint angles
		//computed dx for each time step
		std::vector<Eigen::VectorXd> _dxComputed;
		//computed x position for each step
		std::vector<Eigen::VectorXd> _xComputed;
		//dtheta at each timestep
		std::vector<Eigen::VectorXd> _dtheta;
		//theta at each timestep
		std::vector<vector<double>> _theta;
		//Jacobian at each timestep
		std::vector<Eigen::MatrixXd> _J;
		//Jacobian Pseudoinverse at each timestep
		std::vector<Eigen::MatrixXd> _JPinv;

        //helper functions
        void computeDerived(const vector<double> &initial_joints);
		void clearAll();
        double velocityCost(const Eigen::VectorXd &jointVelocity);
		double positionCost(const Eigen::VectorXd &suggestedPosition, const Eigen::VectorXd &desiredPosition);
		MatrixXd diffFrameCompute(const Eigen::MatrixXd startO, const Eigen::MatrixXd diffO, const Eigen::VectorXd startP, const Eigen::VectorXd diffP);
};
#endif //COSTINEQCOMP_H