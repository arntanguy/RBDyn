#pragma once

#include <RBDyn/parsers/api.h>

#include <SpaceVecAlg/SpaceVecAlg>
#include <RBDyn/Joint.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>
#include <Eigen/Core>
#include <string>

#include <boost/variant.hpp>

namespace rbd {

enum class ParserInput {
	File,
	Content
};

struct RBDYNPARSERS_API Limits
{
public:
	std::map<std::string, std::vector<double> > lower;
	std::map<std::string, std::vector<double> > upper;
	std::map<std::string, std::vector<double> > velocity;
	std::map<std::string, std::vector<double> > torque;
};

struct RBDYNPARSERS_API Geometry
{
public:
	struct Mesh
	{
		Mesh() : scale(1) {
		}
		std::string filename;
		double scale;
	};
	struct Box
	{
		Box() : size(Eigen::Vector3d::Zero()) {
		}
		Eigen::Vector3d size;
	};
	struct Cylinder
	{
		Cylinder() : radius(0.), length(0.) {
		}
		double radius;
		double length;
	};
	struct Sphere
	{
		Sphere() : radius(0.) {
		}
		double radius;
	};
	struct Superellipsoid
	{
		Superellipsoid() : size(Eigen::Vector3d::Zero()), epsilon1(1), epsilon2(1){
		}
		Eigen::Vector3d size;
		double epsilon1,epsilon2;
	};

	enum Type { BOX, CYLINDER, SPHERE, MESH, SUPERELLIPSOID, UNKNOWN };
	Type type;
	boost::variant<Box, Cylinder, Mesh, Sphere, Superellipsoid> data;

	Geometry() : type(UNKNOWN) {
	}
};

struct RBDYNPARSERS_API Visual
{
	std::string name;
	sva::PTransformd origin;
	Geometry geometry;
};


struct RBDYNPARSERS_API ParserResult
{
	rbd::MultiBody mb;
	rbd::MultiBodyConfig mbc;
	rbd::MultiBodyGraph mbg;
	rbd::Limits limits;
	std::map<std::string, std::vector<rbd::Visual> > visual;
	std::map<std::string, std::vector<rbd::Visual> > collision;
	std::map<std::string, sva::PTransformd> collision_tf;
	std::string name;
};

class RBDYNPARSERS_API RBDynParser {
public:
	RBDynParser() = default;
	virtual ~RBDynParser() = default;

	ParserResult& result() {
		return res;
	}

	operator std::tuple<rbd::MultiBody&, rbd::MultiBodyConfig&, rbd::MultiBodyGraph&>() {
		return std::tie(res.mb, res.mbc, res.mbg);
	}

	ParserResult res;
};

}
