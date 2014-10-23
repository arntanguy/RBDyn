// This file is part of RBDyn.
//
// RBDyn is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// RBDyn is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with RBDyn.  If not, see <http://www.gnu.org/licenses/>.

#pragma once

// includes
// std
#include <vector>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

/// The method define bellow allow to estimate the inertial parameter of
/// a rigid body system.
/// We define the inertial parameter of a body i as the 10d vector
/// phi_i = [m, h, I] = [m, hx, hy, hz, Ixx, Ixy, Ixz, Iyy, Iyz, Yzz]


namespace rbd
{
class MultiBody;
class MultiBodyConfig;


/// Return the IMPhi matrix that compute I*m = IMPhi(m)*phi_i.
Eigen::Matrix<double, 6, 10> IMPhi(const sva::MotionVecd& mv);

/// Convert a RBInertiad into a phi vector.
Eigen::Matrix<double, 10, 1> inertiaToVector(const sva::RBInertiad& rbi);

/// Convert a phi vector into a RBInertiad.
sva::RBInertiad vectorToInertia(const Eigen::Matrix<double, 10, 1>& vec);

/** Apply inertiaToVector to all MultiBody Body and concatenate it into one vector
	* Phi = [phi_0, ..., phi_N]
	*/
Eigen::VectorXd multiBodyToInertialVector(const rbd::MultiBody& mb);


/**
	* IDIM stand for Inverse Dynamics Identification Model.
	* This class allow to compute the Y matrix that compute the torque vector with
	* the Phi vector : torque = Y Phi.
	* The Y matrix is the concatenation of Y_{BI} and Y_{JI} matrix in the paper :
	* Study on Dynamics Identification of the Foot Viscoelasticity of a Humanoid Robot
	* Mikami, Yuya
	* Moulard, Thomas
	* Yoshida, Eiichi
	* Venture, Gentiane.
	*/
class IDIM
{
public:
	IDIM(const rbd::MultiBody& mb);

	/**
		* Compute the Y matrix.
		* @param mb MultiBody used has model.
		* @param mbc Use bodyVelB, bodyAccB, parentToSon and motionSubspace.
		* bodyAccB must been calculated with the gravity.
		*/
	void computeY(const rbd::MultiBody& mb, const rbd::MultiBodyConfig& mbc);

	/// Return the Y matrix.
	const Eigen::MatrixXd& Y() const
	{
		return Y_;
	}

private:
	Eigen::MatrixXd Y_;
};

} // rbd
