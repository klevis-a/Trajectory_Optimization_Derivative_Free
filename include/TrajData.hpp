#ifndef TRAJDATA_H
#define TRAJDATA_H
#include <vector>
#include <eigen3/Eigen/Dense>
#include <ll4ma_kdl/manipulator_kdl/robot_kdl.h>

typedef std::vector<Eigen::VectorXd>::size_type VecSz;
typedef std::vector<Eigen::MatrixXd>::size_type MatSz;
typedef std::vector<double>::size_type DblSz;

class TrajData
{
	public:
		//constructor
		TrajData();
		TrajData(const std::string &workSpacePos, const manipulator_kdl::robotKDL kdl);

		//read-only access members
		const double & dt(DblSz i) const;
		const Eigen::VectorXd & dx(VecSz i) const;
		//const Eigen::VectorXd & cdx(VecSz i) const;
		//const Eigen::MatrixXd & ct(MatSz i) const;
		const Eigen::MatrixXd & rotDiff(MatSz i) const;
		const Eigen::VectorXd & transDiff(VecSz i) const;
		double size() const;
		const manipulator_kdl::robotKDL & kdl() const;

		//velocity limits
		static Eigen::VectorXd vel_limits;
		static constexpr double expMultiplier=7.0;
		static constexpr double denominator=400.0;
		static constexpr double orientationScale=60.0;

		//helper function
		Eigen::VectorXd posDiff(const Eigen::VectorXd &start,const Eigen::VectorXd &stop) const;
		Eigen::MatrixXd frameInverse(const vector<double> &pose) const;
		Eigen::MatrixXd rotationDiff(const MatrixXd &initPose, const MatrixXd &finalPose) const;
		Eigen::VectorXd transDiff(const MatrixXd &initPose, const MatrixXd &finalPose) const;

	private:
		//private member variables computed at init
		//time between steps
		std::vector<double> _dt;
		//cumulative dx
		//std::vector<Eigen::VectorXd> _cdx;
		//dx
		std::vector<Eigen::VectorXd> _dx;
		//number of steps
		size_t _size;
		//holds robot KDL
		manipulator_kdl::robotKDL _kdl;
		//holds transformation between each pose
		//std::vector<MatrixXd> _transforms;
		std::vector<MatrixXd> _rotDiff;
		std::vector<VectorXd> _transDiff;
};
#endif //TRAJDATA_H