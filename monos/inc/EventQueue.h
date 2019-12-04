/**
 *  Copyright 2018, 2019 Peter Palfraader
 *					2019 Günther Eder - geder@cs.sbg.ac.at
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <stddef.h>

#include "cgTypes.h"
#include "Heap.h"

class HeapEvent {
public:
	const Event * const e;
	HeapEvent(const Event * p_t);

	const NT& time() const { return e->eventTime; };
	friend std::ostream& operator<<(std::ostream& os, const Event& e);
public:
	CGAL::Comparison_result compare(const HeapEvent &o) const {
			 if (this->time() < o.time()) {
				 return CGAL::SMALLER;
			 } else if (this->time() > o.time()) {
				 return CGAL::LARGER;
			 } else {
				 return CGAL::EQUAL;
			 }
		 }
public:
	 bool operator< (const HeapEvent &o) const { return compare(o) == CGAL::SMALLER; }
	 bool operator> (const HeapEvent &o) const { return compare(o) == CGAL::LARGER; }
	 bool operator>= (const HeapEvent &o) const { return !(*this < o); };
	 bool operator<= (const HeapEvent &o) const { return !(*this > o); };
	 bool operator== (const HeapEvent &o) const { return compare(o) == CGAL::EQUAL; }
	 bool operator!= (const HeapEvent &o) const { return !(*this == o); };
};

class EventQueueItem : public HeapItemBase <HeapEvent> {
private:
public:
	EventQueueItem(const Event * e)
	: HeapItemBase<HeapEvent>(HeapEvent(e))
	{};
};

class EventQueue :  public HeapBase <HeapEvent, EventQueueItem> {
private:
	using Base = HeapBase <HeapEvent, EventQueueItem>;

private:
	std::vector<const Event *> need_update;
	std::vector<const Event *> need_dropping;
	FixedVector<bool> tidx_in_need_dropping;
	FixedVector<bool> tidx_in_need_update;

	FixedVector<ElementType> tidx_to_qitem_map;

	void tidx_to_qitem_map_add(const Event * t, ElementType qi);
	void assert_no_pending() const;
public:
	EventQueue(const Events& events, const Chain& chain);

	void drop_by_tidx(unsigned tidx);
	void update_by_tidx(unsigned tidx);

	/* we /could/ make this const, and our heap a mutable
	 * object attribute and the vectors also mutable.
	 * At which point pretty much everything in here is
	 * declared mutable, so let's just not.
	 */
	const ElementType& peak() const {
		assert_no_pending();
		return Base::peak();
	}
	const ElementType& peak(int idx) const {
		assert_no_pending();
		return Base::peak(idx);
	}
	using Base::size;
	using Base::empty;

	void process_pending_updates();

	void needs_update(const Event * t, bool may_have_valid_collapse_spec = false);
	void needs_dropping(Event * t);

	bool in_needs_update(const ul edgeIdx) const;
	bool in_needs_dropping(const Event * t) const;

	bool is_valid_heap() const;
};
