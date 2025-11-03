#pragma once
#pragma warning(disable : 4996)
#include "SmartRobotDtos.h"

namespace sr
{
	class MyVariant
	{
	public:
		union Value
		{
			Attitude a;
			PathAssignment pa;
			Position p;
			Velocity v;
			Value() {}
			~Value() {}
		} value;

		enum class alternative_t
		{
			attitude, pathassignment, position, velocity, none
		};
		alternative_t alternative;

		MyVariant() : alternative(alternative_t::none), value() {}

		MyVariant(const MyVariant& other) : alternative(other.alternative)
		{
			switch (other.alternative)
			{
			case alternative_t::attitude:
				this->value.a = other.value.a;
				break;
			case alternative_t::pathassignment:
				this->value.pa = other.value.pa;
				break;
			case alternative_t::position:
				this->value.p = other.value.p;
				break;
			case alternative_t::velocity:
				this->value.v = other.value.v;
				break;
			case alternative_t::none:
				break;
			}
		}

		~MyVariant()
		{
			switch (alternative)
			{
			case alternative_t::attitude:
				this->value.a.~Attitude();
				break;
			case alternative_t::pathassignment:
				this->value.pa.~PathAssignment();
				break;
			case alternative_t::position:
				this->value.p.~Position();
				break;
			case alternative_t::velocity:
				this->value.v.~Velocity();
				break;
			case alternative_t::none:
				break;
			}
		}
	};
}