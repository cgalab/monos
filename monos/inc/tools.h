#ifndef TOOLS_H_
#define TOOLS_H_

#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <memory>
#include <assert.h>
#include <ctime>
#include <fstream>

/* 'inspired' by surfer tools.h */

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


#include "cgTypes.h"

bool fileExists(const std::string& fileName);

std::string currentTimeStamp();

void getNormalizer(const BBox& bbox, double& xt, double& xm, double& yt, double& ym, double& zt, double& zm);

void setupEasylogging(int argc, char** argv);
void resetLogging(bool output);

#endif
