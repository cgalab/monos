# MONOS

MONOS computes the straight skeleton of a given monotone polygon.

# Algorithm

The algorithm is based on the following scientific work: https://doi.org/10.1016/j.ipl.2014.09.021
"A simple algorithm for computing positively weighted straight skeletons of monotone polygons"

# Input/Output

Reads GraphML (.graphml) format that describes a polygon. Writes 
.obj format with 3D coordinates which can be imported into programs like
Blender.

# Requirements 
- C++17 enabled complier (gcc,clang)
- CGAL (tested with version 5.0 -- https://doc.cgal.org/5.0/Manual/packages.html) 
- Linux or Mac OS (for Mac OS use -DWITH_GUI=off) 

## Libraries

- MPFR (6, https://www.mpfr.org)
- GMP (10, https://www.mpfr.org)
- BOOST (>= 1.67, with libboost_graph, libboost_regex, https://www.boost.org)

## GUI Libraries

- libQT5Core
- libQT5Xml
- libQT5Gui
- libQT5Widgets
- libQT5Svg
- libQT5=openGL
- libGLX

# CMake Options

In the `CMakeLists.txt` in the main directory contains the following two options 
that can be changed in order to remove the GUI (and QT requirements) or link against
the `CGAL::Cartesian` kernel (faster but inexact). As stated default setting with GUI
and with exact arithmitic.  

`OPTION(WITH_GUI "Enable GUI (requires QT)" 					ON)  # Enabled  by default`
`OPTION(WITH_FP  "Disable exact kernel but use rational kernel" OFF) # Disabled by default`

# Compiling

- git clone --recurse-submodules https://gitlab.cosy.sbg.ac.at/cg/ord/monos.git
- mkdir -p monos/build && cd monos/build
- cmake ..
- make -j 6

# Usage

<code>monos [--verbose][--timings][--normalize] [--out &lt;filename&gt;] &lt;filename&gt;</code>

| options       | shortform | description   |
| -------------:|:---------:|:------------- |
|  --help       | --h       | print help    |
|  --verbose    | --v       | verbose mode, shows information about the computation |
|  --normalize  | --n       | write output normalized to the origin                 |
|  --out        | --o       | write output in wavefront obj format (3D coordinates) |
|  --timings    | --t       | print &lt;vertex count&gt;,&lt;time spent in computation&gt;,&lt;filename&gt;   |
| &lt;filename&gt; | input type is either wavefront obj or GML format   |

Note, the `--verbose` option is only available in the `DEBUG` version.

# Submodules

- easyloggingpp -- https://github.com/weaselp/easyloggingpp


# License
monos is written in C++ and uses CGAL.  It computes the weighted straight
skeleton of a monotone polygon.
Copyright (C) 2018 - Günther Eder - geder@cs.sbg.ac.at

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
