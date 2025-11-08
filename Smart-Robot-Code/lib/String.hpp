#pragma once
#pragma warning(disable : 4996)
#include "Array.hpp"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

namespace sr
{
	namespace details
	{
		inline void dtostrf(double val, int min, int dec, char* buf)
		{
			sprintf(buf, "%.2f", val);
		}

		template<sr::SizeType N>
		class BasicCustomString : protected sr::MyArray<char, N>
		{
		public:
			BasicCustomString() : sr::MyArray<char, N>()
			{
				memset(this->_data, '\0', N);
			}

			BasicCustomString(const BasicCustomString& other) : sr::MyArray<char, N>(other) 
			{
				for(SizeType i = this->_numItems; i < N; ++i)
					this->_data[i] = '\0';
			}

			BasicCustomString(const char* str) : sr::MyArray<char, N>()
			{
				memset(this->_data, '\0', N);
				concat(str);
			}

			BasicCustomString& operator=(const BasicCustomString& other)
			{
				this->clear();
				sr::MyArray<char, N>::operator=(other);
				return *this;
			}
			BasicCustomString& operator=(const char* str)
			{
				this->clear();
				concat(str);
				return *this;
			}

			void operator+=(const double d)
			{
				concat(d);
			}

			void operator+=(const int i)
			{
				concat(i);
			}

			void operator+=(const char c)
			{
				concat(c);
			}

			void operator+=(const BasicCustomString& other)
			{
				concat(other);
			}

			bool operator==(const BasicCustomString& other) const
			{
				return *this == other.c_str();
			}

			bool operator==(const char* str) const
			{
				if (this->length() != strlen(str))
					return false;
				return strcmp(this->_data, str) == 0;
			}

			void concat(const double d)
			{
				char buf[10];
				dtostrf(d, 4, 2, buf);
				add_to_string(buf);
			}

			void concat(const int i)
			{
				char buf[10];
				sprintf(buf, "%d", i);
				add_to_string(buf);
			}

			void concat(const BasicCustomString& other)
			{
				add_to_string(other.c_str());
			}

			void concat(const char* str)
			{
				add_to_string(str);
			}

			void concat(const char c)
			{
				char buf[2] = { c, '\0' };
				add_to_string(buf);
			}

			template<class T>
			T substring(const sr::SizeType start, const sr::SizeType length = N) const
			{
				T result;
				if (start >= this->size() || length == 0)
					return result;

				sr::SizeType actualLength = length;
				if (start + length > this->size())
					actualLength = this->size() - start;

				for (sr::SizeType i = 0; i < actualLength; ++i)
				{
					result.concat(this->_data[start + i]);
				}
				return result;
			}

			const sr::SizeType index_of(const char c, const sr::SizeType start = 0) const
			{
				if (start >= this->size())
					return -1;

				for (sr::SizeType i = start; i < this->size(); ++i)
				{
					if (this->_data[i] == c)
						return i;
				}
				return -1;
			}

			const char char_at(const sr::SizeType index) const
			{
				return this->at(index);
			}

			void clear() override
			{
				sr::MyArray<char, N>::clear();
				memset(this->_data, '\0', N);
			}

			const double to_double() const
			{
				return atof(this->_data);
			}

			const int to_int() const
			{
				return atoi(this->_data);
			}

			constexpr sr::SizeType length() const
			{
				return this->size();
			}

			constexpr sr::SizeType capacity() const
			{
				return N;
			}

			const char* c_str() const
			{
				return this->_data;
			}

		private:
			void add_to_string(const char* buf)
			{
				if (strlen(buf) + this->size() >= N)
					return;

				strcat(this->_data, buf);
				this->_numItems += strlen(buf);
			}
		};
	}
	using BigString = details::BasicCustomString<1000>;
	using DefaultString = details::BasicCustomString<500>;
	using SmallString = details::BasicCustomString<400>;
	using HeaderString = details::BasicCustomString<100>;
	using SplitsString = details::BasicCustomString<30>;
	using TinyString = details::BasicCustomString<60>;
}