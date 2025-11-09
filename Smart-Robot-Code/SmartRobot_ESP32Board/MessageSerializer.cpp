#include "MessageSerializer.h"
#include "Split.h"

namespace sr
{
	DefaultString other;
	MyArray<SplitsString, 2> headerParts;
	MyArray<SplitsString, 6> waypointParts;
	MyArray<SplitsString, 3> distanceParts;
	Waypoint wp;

	bool deserializeRobotMessage(const DefaultString& msg, RobotMessageData& outObj)
	{
		try
		{
			split<2>(msg.substring<HeaderString>(1, msg.index_of('}') - 1), headerParts);
			if (headerParts.size() < 2)
			{
				goto clearandend;
			}

			if (headerParts[1] == "Standby")
			{
				outObj.type = MsgFromRobotType::Standby;
			}
			else if (headerParts[1] == "Distance")
			{
				outObj.type = MsgFromRobotType::Distance;
				other = msg.substring<DefaultString>(msg.index_of('}') + 1);
				split<3>(other.substring<HeaderString>(0), distanceParts);
				outObj.distance = distanceParts[0].to_double();
				outObj.velocity = distanceParts[1].to_double();
				outObj.heading = distanceParts[2].to_double();
			}
			else
			{
				outObj.type == MsgFromRobotType::None;
			}
		clearandend:
			distanceParts.clear();
			headerParts.clear();
			other.clear();
		}
		catch (...) { return false; }

		return true;
	}

	void deserializePathAssignment(const DefaultString& msg, PathAssignment& pa)
	{
		SplitsString part = msg.substring<SplitsString>(0, msg.index_of(','));
		pa.pathID = part.to_int();

		unsigned int currentIndex = msg.index_of(',') + 1;
		while (currentIndex < msg.length())
		{
			if (msg.char_at(currentIndex) == '[')
			{
				split<6>(msg.substring<HeaderString>(currentIndex, msg.index_of(']', currentIndex) - currentIndex), waypointParts);

				wp.pointID = waypointParts[0].to_int();
				wp.desiredVelocity = waypointParts[1].to_double();
				wp.point.x = waypointParts[2].to_double();
				wp.point.y = waypointParts[3].to_double();
				wp.point.z = waypointParts[4].to_double();
				wp.heading = waypointParts[5].to_double();

				pa.waypoints.insert_back(wp);

				currentIndex = msg.index_of(']', currentIndex);
				waypointParts.clear();
			}
			++currentIndex;
		}
		waypointParts.clear();
	}

	void deserialize(const DefaultString& msg, MyVariant& outObj)
	{
		if (msg.length() > 0)
			split<2>(msg.substring<HeaderString>(1, msg.index_of('}') - 1), headerParts);

		if (headerParts[1] == "PathAssignment")
		{
			outObj.value.pa = PathAssignment();
			other = msg.substring<DefaultString>(msg.index_of('}') + 1);

			outObj.alternative = MyVariant::alternative_t::pathassignment;
			deserializePathAssignment(other, outObj.value.pa);
		}
		else if (headerParts[1] == "ClearAssignment")
		{
			outObj.alternative = MyVariant::alternative_t::stop;
		}
		else
		{
			outObj.alternative = MyVariant::alternative_t::none;
		}
		headerParts.clear();
		other.clear();
	}

	void serializeWaypoints(const PathAssignment& obj, DefaultString& outMsg)
	{
		const Waypoint* wp = nullptr;
		outMsg = "{Smart Robot,PathAssignment}";
		outMsg += (int)obj.pathID;
		for (unsigned int i = 0; i < obj.waypoints.size(); ++i)
		{
			wp = &obj.waypoints.at(i);

			outMsg += ",[";
			outMsg += (int)wp->pointID;
			outMsg += ",";
			outMsg += wp->desiredVelocity;
			outMsg += ",";
			outMsg += wp->point.x;
			outMsg += ",";
			outMsg += wp->point.y;
			outMsg += ",";
			outMsg += wp->point.z;
			outMsg += ",";
			outMsg += wp->heading;
			outMsg += "]";
		}
	}

	void serializeAsset(const SmartRobotAsset& obj, DefaultString& outMsg)
	{
		outMsg = "{Smart Robot,SmartRobotAsset}";
		outMsg += obj.name;
		outMsg += ",";
		outMsg += obj.front;
		outMsg += ",";
		outMsg += obj.back;
		outMsg += ",";
		outMsg += obj.left;
		outMsg += ",";
		outMsg += obj.right;
		outMsg += ",";
		outMsg += obj.height;
	}

	void serializeVelocity(const Velocity& obj, DefaultString& outMsg)
	{
		outMsg = "{Smart Robot,Velocity}";
		outMsg += obj.x;
		outMsg += ",";
		outMsg += obj.y;
		outMsg += ",";
		outMsg += obj.z;
		outMsg += ",";
		outMsg += obj.rollRate;
		outMsg += ",";
		outMsg += obj.pitchRate;
		outMsg += ",";
		outMsg += obj.yawRate;
	}

	void serializePosition(const Position& obj, DefaultString& outMsg)
	{
		outMsg = "{Smart Robot,Position}";
		outMsg += obj.x;
		outMsg += ",";
		outMsg += obj.y;
		outMsg += ",";
		outMsg += obj.z;
	}

	void serializeAttitude(const Attitude& obj, DefaultString& outMsg)
	{
		outMsg = "{Smart Robot,Attitude}";
		outMsg += obj.roll;
		outMsg += ",";
		outMsg += obj.pitch;
		outMsg += ",";
		outMsg += obj.yaw;
	}

	void serializeCurrentAssignment(const CurrentAssignment& obj, DefaultString& outMsg)
	{
		outMsg = "{Smart Robot,CurrentAssignment}";
		outMsg += (int)obj.pathID;
	}

	void serializeCurrentSegment(const CurrentSegment& obj, DefaultString& outMsg)
	{
		outMsg = "{Smart Robot,CurrentSegment}";
		outMsg += (int)obj.pointID;
	}
}