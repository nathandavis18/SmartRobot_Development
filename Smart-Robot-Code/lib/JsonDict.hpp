#pragma once
#include "Array.hpp"
#include "String.hpp"
using JsonStringKey = details::BasicCustomString<10>;
using JsonString = details::BasicCustomString<200>;

namespace sr
{
	namespace details
	{
		struct KeyValuePair
		{
			KeyValuePair() : key(), value(0.0) {}
			KeyValuePair(const JsonStringKey& k, const double v) : key(k), value(v) {}

			bool operator==(const KeyValuePair& other) const
			{
				return key == other.key && value == other.value;
			}

			JsonStringKey key;
			double value;
		};

		void get_key(const JsonString& str, SizeType& index, JsonStringKey& key)
		{
			++index;
			while (index < str.length() && str.char_at(index) != '"')
			{
				key += str.char_at(index);
				++index;
			}
			while (str.char_at(index) != ':' && index < str.length())
				++index;
			++index; // Move past the colon
		}

		void get_value(const JsonString& str, SizeType& index, double& value)
		{
			JsonString valueStr;
			while (index < str.length() && (str.char_at(index) == ' ' || str.char_at(index) == '\n' || str.char_at(index) == '\r'))
				++index;
			while (index < str.length() && ((str.char_at(index) >= '0' && str.char_at(index) <= '9') || str.char_at(index) == '.' || str.char_at(index) == '-'))
			{
				valueStr += str.char_at(index);
				++index;
			}
			value = valueStr.to_double();
		}

		template<SizeType N>
		class JsonDict : protected MyArray<details::KeyValuePair, N>
		{
		public:
			double operator[](const JsonStringKey& key) const
			{
				for (SizeType i = 0; i < this->size(); ++i)
				{
					if (this->_data[i].key == key)
						return this->_data[i].value;
				}
				return 0;
			}
			void insert_back(const KeyValuePair& key)
			{
				if (this->contains(key))
					return;
				MyArray<details::KeyValuePair, N>::insert_back(key);
			}
			bool contains_key(const JsonStringKey& key) const
			{
				for (SizeType i = 0; i < this->size(); ++i)
				{
					if (this->_data[i].key == key)
						return true;
				}
				return false;
			}
			void clear() override
			{
				MyArray<details::KeyValuePair, N>::clear();
			}
		};
	}

	using MyDictionary = details::JsonDict<20>;

	void parse_json(const JsonString& str, MyDictionary& dict)
	{
		SizeType i = 0;
		if (str.char_at(i) != '{') return;
		JsonStringKey currentKey;
		double currentValue;
		for (i = 1; i < str.length(); ++i)
		{
			while (str.char_at(i) != '\"' && i < str.length())
				++i;

			if (i >= str.length())
				break;

			details::get_key(str, i, currentKey);
			details::get_value(str, i, currentValue);
			dict.insert_back(details::KeyValuePair(currentKey, currentValue));

			currentKey.clear();
			currentValue = 0;
		}
	}
}