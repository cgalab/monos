#pragma once

#include <iostream>

#include "cgTypes.h"

class EdgeCollapseSpec {
private:
	NT time_;

public:
	EdgeCollapseSpec() {}
	EdgeCollapseSpec(const NT& time)
	: time_(time)
	{
	};

	const NT& time() const { return time_; };
	double get_printable_time() const { return CGAL::to_double(time_); }
};
std::ostream& operator<<(std::ostream& os, const EdgeCollapseSpec& s);


class CollapseSpec {
public:
	static unsigned COUNTER_NT_cmp;

private:
	NT time_;
public:
	CollapseSpec& operator =(const CollapseSpec& o) {
		time_ = o.time_;
		return *this;
	}


	CollapseSpec(const NT& time)
	: time_(time)
	{
	};

	 const NT& time() const { return time_; };
	 double get_printable_time() const { return CGAL::to_double(time_); }


private:
	 CGAL::Comparison_result compare(const CollapseSpec &o) const {
		 if (this->time() < o.time()) {
			 return CGAL::SMALLER;
		 } else if (this->time() > o.time()) {
			 return CGAL::LARGER;
		 } else {
			 return CGAL::EQUAL;
		 }
	 }

public:
	 bool operator< (const CollapseSpec &o) const { return compare(o) == CGAL::SMALLER; }
	 bool operator> (const CollapseSpec &o) const { return compare(o) == CGAL::LARGER; }
	 bool operator>= (const CollapseSpec &o) const { return !(*this < o); };
	 bool operator<= (const CollapseSpec &o) const { return !(*this > o); };
	 bool operator== (const CollapseSpec &o) const { return compare(o) == CGAL::EQUAL; }
	 bool operator!= (const CollapseSpec &o) const { return !(*this == o); };
};
std::ostream& operator<<(std::ostream& os, const CollapseSpec& s);
