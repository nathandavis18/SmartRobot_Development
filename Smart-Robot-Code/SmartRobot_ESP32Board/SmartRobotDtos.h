#pragma once
#pragma warning(disable : 4996)

#include "../lib/Array.hpp"

namespace sr
{
	struct Attitude
	{
		double roll, pitch, yaw;
		Attitude() : roll(0), pitch(0), yaw(0) {}
	};

	struct Position
	{
		double x, y, z;
		Position() : x(0), y(0), z(0) {}
	};

	struct Waypoint
	{
		unsigned int pointID;
		double desiredVelocity;
		Position point;
		double heading;

		Waypoint() : pointID(0), desiredVelocity(0), point(Position()), heading(0) {}
	};

	struct PathAssignment
	{
		unsigned long pathID;
		MyArray<Waypoint, 20> waypoints;

		PathAssignment() : pathID(0), waypoints(MyArray<Waypoint, 20>()) {}
	};

	struct SmartRobotAsset
	{
		static constexpr char name[] = "Smart Robot";
		static constexpr double front = 3, back = 3, left = 3, right = 3, height = 1.5;
	};

	struct Velocity
	{
		double x, y, z;
		double rollRate, pitchRate, yawRate;

		Velocity() : x(0), y(0), z(0), rollRate(0), pitchRate(0), yawRate(0) {}
	};

	struct CurrentAssignment
	{
		unsigned long pathID;
	};

	struct CurrentSegment
	{
		unsigned long pointID;
	};
}