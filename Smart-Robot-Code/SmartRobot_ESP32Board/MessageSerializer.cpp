#include <exception>

#include "MessageSerializer.h"
#include "Split.h"

namespace sr
{
	MyString header, other;
	MyArray<MyString, 2> headerParts;
	MyArray<MyString, 6> waypointParts;
	Waypoint wp;

	bool deserializeRobotMessage(const MyString& msg, RobotMessageData& outObj)
	{
		try
		{
			header = msg.substring(1, msg.index_of('}') - 1);

			split<2>(header, headerParts);
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
				other = msg.substring(msg.index_of('}') + 1);
				outObj.distance = other.to_double();
			}
			else
			{
				outObj.type == MsgFromRobotType::None;
			}
		clearandend:
			headerParts.clear();
			header.clear();
			other.clear();
		}
		catch (std::exception ex) { return false; }

		return true;
	}

	void deserializePathAssignment(const MyString& msg, PathAssignment& pa)
	{
		pa.pathID = msg.substring(0, msg.index_of(',')).to_int();

		unsigned int currentIndex = msg.index_of(',') + 1;
		while (currentIndex < msg.length())
		{
			if (msg.char_at(currentIndex) == '[')
			{
				split<6>(msg.substring(currentIndex, msg.index_of(']', currentIndex) - currentIndex), waypointParts);

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

	void deserialize(const MyString& msg, MyVariant& outObj)
	{
		header = msg.substring(1, msg.index_of('}') - 1);

		split<2>(header, headerParts);

		if (headerParts[1] == "PathAssignment")
		{
			outObj.value.pa = PathAssignment();
			other = msg.substring(msg.index_of('}') + 1);

			outObj.alternative = MyVariant::alternative_t::pathassignment;
			deserializePathAssignment(other, outObj.value.pa);
		}
		else
		{
			outObj.alternative = MyVariant::alternative_t::none;
		}
		headerParts.clear();
		header.clear();
		other.clear();
	}

	void serializeWaypoints(const PathAssignment& obj, MyString& outMsg)
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

	void serializeAsset(const SmartRobotAsset& obj, MyString& outMsg)
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

	void serializeVelocity(const Velocity& obj, MyString& outMsg)
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

	void serializePosition(const Position& obj, MyString& outMsg)
	{
		outMsg = "{Smart Robot,Position}";
		outMsg += obj.x;
		outMsg += ",";
		outMsg += obj.y;
		outMsg += ",";
		outMsg += obj.z;
	}

	void serializeAttitude(const Attitude& obj, MyString& outMsg)
	{
		outMsg = "{Smart Robot,Attitude}";
		outMsg += obj.roll;
		outMsg += ",";
		outMsg += obj.pitch;
		outMsg += ",";
		outMsg += obj.yaw;
	}

	void serializeCurrentAssignment(const CurrentAssignment& obj, MyString& outMsg)
	{
		outMsg = "{Smart Robot,CurrentAssignment}";
		outMsg += (int)obj.pathID;
	}

	void serializeCurrentSegment(const CurrentSegment& obj, MyString& outMsg)
	{
		outMsg = "{Smart Robot,CurrentSegment}";
		outMsg += (int)obj.pointID;
	}
}