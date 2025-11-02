#pragma once
#pragma warning(disable : 4996)

#include "SmartRobotDtos.h"
namespace sr{
  class MyVariant{
  public:
    union {
      Attitude a;
      PathAssignment pa;
      Position p;
      Velocity v;
    };

    enum class alternative_t{
      attitude, pathassignment, position, velocity, none
    };
    alternative_t alternative;

    MyVariant() : alternative(alternative_t::none) {}

    MyVariant(const MyVariant& other) : alternative(other.alternative) {
      switch(other.alternative){
        case alternative_t::attitude:
          this->a = other.a;
          break;
        case alternative_t::pathassignment:
          this->pa = other.pa;
          break;
        case alternative_t::position:
          this->p = other.p;
          break;
        case alternative_t::velocity:
          this->v = other.v;
          break;
        case alternative_t::none:
          break;
      }
    }

    ~MyVariant() {
      switch(alternative){
        case alternative_t::attitude:
          this->a.~Attitude();
          break;
        case alternative_t::pathassignment:
          this->pa.~PathAssignment();
          break;
        case alternative_t::position:
          this->p.~Position();
          break;
        case alternative_t::velocity:
          this->v.~Velocity();
          break;
        case alternative_t::none:
          break;
      }
    }
  };
}