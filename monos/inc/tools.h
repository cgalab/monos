/*
 * monos is written in C++.  It computes the weighted straight skeleton
 * of a monotone polygon in asymptotic n log n time and linear space.
 * Copyright (C) 2018 - Günther Eder - geder@cs.sbg.ac.at
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef TOOLS_H_
#define TOOLS_H_

#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <memory>
#include <assert.h>
#include <ctime>
#include <fstream>
#include <iostream>
#include <vector>

/* 'inspired' by surfer tools.h */
static inline int log2i(unsigned v) {
	int r = -1;
	while (v > 0) {
		r++;
		v >>= 1;
	};
	return r;
}

template <class T>
void sort_tuple(T& a, T& b) {
	if (a>b) std::swap(a,b);
}

template <class T>
std::tuple<unsigned, unsigned, unsigned> indirect_sort_3(const T t[3]) {
  unsigned i0 = 0;
  unsigned i1 = 1;
  unsigned i2 = 2;
  if (t[i0] > t[i1]) std::swap(i0, i1);
  if (t[i1] > t[i2]) std::swap(i1, i2);
  if (t[i0] > t[i1]) std::swap(i0, i1);
  return std::make_tuple(i0, i1, i2);
}

template <typename T>
static T compute_determinant(const T& x0, const T& y0,
                             const T& x1, const T& y1,
                             const T& x2, const T& y2) {
  return T(
    ( x0 * y1
    + x1 * y2
    + x2 * y0
    )
    -
    ( y0 * x1
    + y1 * x2
    + y2 * x0
    )
  );
}

template <class T>
struct FixedVector
		: private std::vector<T> {
		private:
	using Base = std::vector<T>;
	using size_type = typename Base::size_type;
	using value_type = typename Base::value_type;

	using Base::capacity;
		public:
	using const_iterator = typename Base::const_iterator;

	void reserve(size_type new_size) {
		assert(size() == 0);
		Base::reserve(new_size);
	}
	void resize(size_type new_size, const value_type& val) {
		assert(size() == 0);
		Base::resize(new_size, val);
	}
	//using Base::vector;
	//using Base::operator=;
	//using Base::get_allocator;
	using Base::at;
	//using Base::front;
	using Base::back;
	//using Base::clear;
	//using Base::data;
	using Base::begin;
	//using Base::cbegin
	using Base::end;
	//using Base::cend;
	//using Base::empty;
	using Base::size;
	using Base::operator[];

	void emplace_back (value_type&& val) {
		assert(size() < capacity());
		Base::emplace_back(std::forward<value_type>(val));
	}
	void push_back (const value_type& val) {
		assert(size() < capacity());
		Base::push_back(val);
	}
	void push_back (value_type&& val) {
		assert(size() < capacity());
		Base::push_back(std::forward<value_type>(val));
	}
};

bool fileExists(const std::string& fileName);

std::string currentTimeStamp();


void setupEasylogging(int argc, char** argv);
void resetLogging(bool output);

#endif
