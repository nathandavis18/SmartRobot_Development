#pragma once
#pragma warning(disable : 4996)

#include <type_traits>

#include "MyVariant.h"
#include "SmartRobotDtos.h"
#include "../lib/String.hpp"

namespace sr{

  enum class MsgFromRobotType{
    Standby,
    Distance,
    Driving,
    None
  };

  struct RobotMessageData{
    MsgFromRobotType type = MsgFromRobotType::None;
    double distance;
    void clear() {
      distance = 0;
      type = MsgFromRobotType::None;
    }
  };

  void serializeWaypoints(const PathAssignment& obj, MyString& outMsg);

  void serializeAsset(const SmartRobotAsset& obj, MyString& outMsg);

  void serializeVelocity(const Velocity& obj, MyString& outMsg);

  void serializePosition(const Position& obj, MyString& outMsg);

  void serializeAttitude(const Attitude& obj, MyString& outMsg);

  void serializeCurrentAssignment(const CurrentAssignment& obj, MyString& outMsg);

  void serializeCurrentSegment(const CurrentSegment& obj, MyString& outMsg);

  void deserialize(const MyString& msg, MyVariant& outObj);

  template<class>
  struct always_false : std::false_type {};

  //This is a template so it has to be defined in the header. The rest are in the .cpp file
  template<class T>
  void serialize(const T& obj, MyString& outMsg)
  {
    if constexpr(std::is_same_v<T, Attitude>){
        serializeAttitude(obj, outMsg);
    }
    else if constexpr(std::is_same_v<T, PathAssignment>){
        serializeWaypoints(obj, outMsg);
    }
    else if constexpr(std::is_same_v<T, Position>){
		    serializePosition(obj, outMsg);
    }
    else if constexpr(std::is_same_v<T, Velocity>){
		    serializeVelocity(obj, outMsg);
    }
    else if constexpr(std::is_same_v<T, CurrentAssignment>){
        serializeCurrentAssignment(obj, outMsg);
    }
    else if constexpr(std::is_same_v<T, CurrentSegment>){
        serializeCurrentSegment(obj, outMsg);
    }
    else if constexpr(std::is_same_v<T, SmartRobotAsset>){
        serializeAsset(obj, outMsg);
    }
    else{
      static_assert(always_false<T>::value, "Unsupported type for serialization");
    }
  }

  bool deserializeRobotMessage(const MyString& msg, RobotMessageData& outObj);
}