# MONOS

MONOS computes the straight skeleton of a given monotone polygon.

# Algorithm

The algorithm is based on the following scientific work: https://doi.org/10.1016/j.ipl.2014.09.021
-- A simple algorithm for computing positively weighted straight skeletons of monotone polygons.

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

	OPTION(WITH_GUI "Enable GUI (requires QT)" ON) # Enabled  by default
	OPTION(WITH_FP  "Disable exact kernel but use rational kernel" OFF) # Disabled by default

# Compiling

	git clone --recurse-submodules git@github.com:cgalab/monos.git
	mkdir -p monos/build && cd monos/build
	cmake .. 
	make -j 6

For the `RELEASE` version use `cmake -D CMAKE_BUILD_TYPE=Release ..`

# Usage

	monos [--verbose][--timings][--normalize] --out &lt;filename&gt; &lt;filename&gt;

| options       | shortform | description   |
| -------------:|----------:|:------------- |
|  --help       | --h       | print help    |
|  --verbose    | --v       | verbose mode, shows information about the computation |
|  --normalize  | --n       | write output normalized to the origin                 |
|  --out        | --o       | write output in wavefront obj format (3D coordinates) |
|  --timings    | --t       | print &lt;vertex count&gt;,&lt;time spent in computation&gt;,&lt;memuse&gt;,&lt;filename&gt;   |
| &lt;filename&gt; | | input type is either wavefront obj or GML format   |

Note, the `--verbose` option is only available in the `DEBUG` version.

Monos reads GraphML format. Using [format-converter](https://github.com/cgalab/format-converter) may common
formats can be converted into GraphML.

# Submodules

- easyloggingpp -- https://github.com/cgalab/easyloggingpp

# Code Structure

Directories `cc` and `gui` produces the binaraies for the CLI and the GUI version.
Tha main algorithm and library is in `monos` sparated in `src` and `inc` directory.

|     File    | Discription   |
| -----------:|:------------- |
|  Wavefront   | Compute the Straight Skeleton of the two monotone chains. Holds the Nodes and Arcs that store the straight skeleton. |
|  Skeleton   | Construct the Merge and joins the two skeletons. |
| BasicInput, BGLGraph|  Read the .graphml input file. |


# License

Monos is licenced under [GPLv3](https://www.gnu.org/licenses/gpl-3.0.html).


