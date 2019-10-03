#pragma once

#include <yaml-cpp/yaml.h>

#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyGraph.h>
#include <RBDyn/MultiBodyConfig.h>

#include <RBDyn/parsers/common.h>

namespace rbd {

class RBDYNPARSERS_API RBDynFromYAML : public RBDynParser {
public:
	RBDynFromYAML(const std::string & input,
	              ParserInput input_type = ParserInput::File,
	              bool fixed = true,
	              const std::vector<std::string> & filtered_links = {},
	              bool transform_inertia = true,
	              const std::string & base_link = "",
	              bool with_virtual_links = true,
	              const std::string spherical_suffix = "_spherical");

	 virtual ~RBDynFromYAML() = default;

private:
	bool verbose_;
	bool transform_inertia_;
	bool angles_in_degrees_;
	std::string base_link_;
	size_t link_idx_;
	size_t joint_idx_;
	std::map<std::string, rbd::Joint::Type> joint_types_;
	std::vector<std::string> filtered_links_;
	bool with_virtual_links_;
	const std::string& spherical_suffix_;

	Eigen::Matrix3d makeInertia(double ixx, double iyy, double izz, double iyz, double ixz, double ixy);

	Eigen::Matrix3d MatrixFromRPY(double r, double p, double y);

	void parseFrame(const YAML::Node& frame, const std::string& name, Eigen::Vector3d& xyz, Eigen::Vector3d& rpy);

	void parseInertia(const YAML::Node& inertia, Eigen::Matrix3d& inertia_mat);

	void parseInertial(const YAML::Node& inertial, const std::string& name, double& mass, Eigen::Vector3d& xyz, Eigen::Vector3d& rpy, Eigen::Matrix3d& inertia);

	bool parseGeometry(const YAML::Node& geometry, rbd::Geometry& data);

	void parseVisuals(const YAML::Node& visuals, std::vector<rbd::Visual>& data);

	void parseLink(YAML::Node link);

	bool parseJointType(const YAML::Node& type, const std::string& name, rbd::Joint::Type& joint_type, std::string& type_name);

	void parseJointAxis(const YAML::Node& axis, const std::string& name, Eigen::Vector3d& joint_axis);

	void parseJointLimits(const YAML::Node& limits, const std::string& name, const rbd::Joint& joint, bool is_continuous);

	void parseJoint(YAML::Node joint);
};

} // namespace rbd
