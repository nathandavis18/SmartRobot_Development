#pragma once
#pragma warning(disable : 4996)

#include "../lib/String.hpp"
#include "../lib/Array.hpp"

namespace sr
{
	template<unsigned int size>
	void split(const HeaderString& data, MyArray<SplitsString, size>& arr, char delimiter = ',')
	{
		unsigned int currentPos = 0;
		if (data.char_at(currentPos) == '[' || data.char_at(currentPos) == '{')
			++currentPos;

		unsigned int dataBegin = currentPos;
		while (currentPos < data.length())
		{
			currentPos = data.index_of(delimiter, currentPos);

			if (currentPos != dataBegin && currentPos < data.length())
			{
				arr.insert_back(data.substring(dataBegin, currentPos - dataBegin));
				++currentPos;
				dataBegin = currentPos; // Move past the delimiter
			}
		}

		currentPos = data.length();
		if (data.char_at(data.length() - 1) == ']' || data.char_at(data.length() - 1) == '}')
		{
			currentPos = data.length() - 1;
		}

		if (currentPos != dataBegin)
		{
			arr.insert_back(data.substring(dataBegin, currentPos - dataBegin));
		}
	}
}