#pragma once
#pragma warning(disable : 4996)

namespace sr
{
	using SizeType = unsigned int;
	template<typename T, SizeType N>
	class MyArray
	{
	public:
		MyArray() : _numItems(0), _data({ T() }) {}
		MyArray(const MyArray& other) : _numItems(other._numItems)
		{
			for (SizeType i = 0; i < _numItems; ++i)
			{
				_data[i] = other._data[i];
			}
		}
		MyArray& operator=(const MyArray& other)
		{
			if (this != &other)
			{
				_numItems = other._numItems;
				for (SizeType i = 0; i < _numItems; ++i)
				{
					_data[i] = other._data[i];
				}
			}
			return *this;
		}

		bool insert_back(const T& value)
		{
			if (_numItems >= N)
				return false;
			_data[_numItems++] = value;
			return true;
		}

		bool insert_at(const T& value, SizeType index)
		{
			if (_numItems >= N || index > _numItems)
				return false;

			for (SizeType i = _numItems; i > index; --i)
			{
				_data[i] = _data[i - 1];
			}
			_data[index] = value;
			++_numItems;
			return true;
		}

		void pop_back()
		{
			if (_numItems == 0)
				return;
			_data[--_numItems].~T();
		}

		void remove_at(SizeType index)
		{
			if (index >= _numItems)
				return;
			if (index == _numItems - 1)
			{
				pop_back();
				return;
			}

			_data[index].~T();
			for (SizeType i = index; i < _numItems - 1; ++i)
			{
				_data[i] = _data[i + 1];
			}
			--_numItems;
		}

		virtual void clear()
		{
			while (_numItems > 0)
				pop_back();
		}

		T& at(SizeType index)
		{
			if (index >= _numItems)
				throw "Bad";
			return _data[index];
		}

		const T& at(SizeType index) const
		{
			if (index >= _numItems)
				throw "Bad";
			return _data[index];
		}

		T& operator[](SizeType index)
		{
			return _data[index];
		}

		const T& operator[](SizeType index) const
		{
			return _data[index];
		}

		const T& first() const
		{
			if (_numItems == 0)
				throw "Bad";
			return _data[0];
		}

		const T& last() const
		{
			if (_numItems == 0)
				throw "Bad";
			return _data[_numItems - 1];
		}

		bool contains(const T& val) const
		{
			for (SizeType i = 0; i < _numItems; ++i)
			{
				if (_data[i] == val)
					return true;
			}
			return false;
		}

		constexpr SizeType size() const
		{
			return _numItems;
		}

		constexpr SizeType capacity() const
		{
			return N;
		}

		bool operator==(const MyArray<T, N>& other) const
		{
			if (_numItems != other._numItems)
				return false;
			for (SizeType i = 0; i < _numItems; ++i)
			{
				if (!(_data[i] == other._data[i]))
					return false;
			}
			return true;
		}

	protected:
		T _data[N];
		SizeType _numItems;
	};
}