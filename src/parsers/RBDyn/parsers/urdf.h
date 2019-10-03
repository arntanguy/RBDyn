/* Copyright 2015-2017 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is part of mc_rbdyn_urdf.
 *
 * mc_rbdyn_urdf is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * mc_rbdyn_urdf is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with mc_rbdyn_urdf.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <SpaceVecAlg/SpaceVecAlg>
#include <RBDyn/Joint.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>
#include <Eigen/Core>
#include <tinyxml2.h>
#include <string>

#include <RBDyn/parsers/common.h>

namespace rbd
{

class RBDYNPARSERS_API RBDynFromURDF : public RBDynParser {
public:
	RBDynFromURDF(
		const std::string & input,
		ParserInput input_type = ParserInput::File,
		bool fixed = true,
		const std::vector<std::string> & filteredLinksIn = {},
		bool transformInertia = true,
		const std::string & baseLinkIn = "",
		bool withVirtualLinks = true,
		const std::string sphericalSuffix = "_spherical");

 virtual ~RBDynFromURDF() = default;

};

std::vector<double> attrToList(const tinyxml2::XMLElement & dom, const std::string & attr, const std::vector<double> & def = {});

Eigen::Vector3d attrToVector(const tinyxml2::XMLElement & dom, const std::string & attr, const Eigen::Vector3d & def = Eigen::Vector3d(0,0,0));

Eigen::Matrix3d RPY(const double & r, const double & p, const double & y);

rbd::Joint::Type rbdynFromUrdfJoint(const std::string & type);

sva::PTransformd originFromTag(const tinyxml2::XMLElement & root, const std::string & tagName);
sva::PTransformd originFromTag(const tinyxml2::XMLElement * dom);

ParserResult rbdyn_from_urdf(const std::string & content, bool fixed = true, const std::vector<std::string> & filteredLinksIn = {}, bool transformInertia = true, const std::string & baseLinkIn = "", bool withVirtualLinks = true, const std::string sphericalSuffix = "_spherical");

std::string parseMultiBodyGraphFromURDF(ParserResult& res, const std::string & content, const std::vector<std::string> & filteredLinksIn = {}, bool transformInertia = true, const std::string & baseLinkIn = "", bool withVirtualLinks = true, const std::string sphericalSuffix = "_spherical");

}
