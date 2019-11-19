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

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

#include <iostream>
#include <vector>
#include <assert.h>

#include <limits>

#include "../easyloggingpp/src/easylogging++.h"


#define NIL    -1
#define ZSCALE 0.5
#define OBJSCALE 10

#define smallEPS 0.00000001

/*
 * OBJ files index starting at one, we read the input
 * and store the vertices starting at 0, thus we change
 * the reference from faces ot vertex by this offset
 **/
#define OBJOFFSET -1

#define BOOST_VARIANT_USE_RELAXED_GET_BY_DEFAULT

#define INFPOINT Point(std::numeric_limits<double>::max(),std::numeric_limits<double>::max())

using ul = unsigned long;
using sl = signed long;

enum class NodeType   		: ul {TERMINAL=0,NORMAL,DISABLED};
enum class ArcType    		: ul {NORMAL=0,RAY,DISABLED};
enum class BisType    	   	: ul {RAY,LINE};
enum class MonotoneType    	: ul {START=0,END};
enum class ChainType    	: ul {UPPER=0,LOWER};

#define MAX std::numeric_limits<sl>::max()

#endif
