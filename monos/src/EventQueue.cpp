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

#include "EventQueue.h"

HeapEvent::
HeapEvent(const Event *  p_t)
: e(p_t)
{
}

EventQueue::
EventQueue(const Events& setEvents, const Chain& chain):events(setEvents) {
	ArrayType a;
	tidx_to_qitem_map.resize(events.size(), NULL);
	tidx_in_need_dropping.resize(events.size(), false);
	tidx_in_need_update.resize(events.size(), false);

	/* we skip the first and last edge of each chain */
	for (auto t = std::next(chain.begin()); t != std::prev(chain.end()); ++t) {
		auto qi = std::make_shared<EventQueueItem>(&events[*t]);
		a.emplace_back(qi);
		tidx_to_qitem_map_add(&events[*t], qi);
	}
	setArray(a);
}

void
EventQueue::
tidx_to_qitem_map_add(const Event * t, ElementType qi) {
	unsigned id = t->mainEdge;
	assert(id < tidx_to_qitem_map.size());
	tidx_to_qitem_map[id] = qi;
}

void
EventQueue::
drop_by_tidx(unsigned tidx) {
	LOG(INFO) << "___ dropping idx: " << tidx;
	auto qi = tidx_to_qitem_map.at(tidx);
	assert(NULL != qi);
	drop_element(qi);
	tidx_to_qitem_map[tidx] = NULL;
	tidx_in_need_dropping[tidx] = false;
}

void
EventQueue::
update_by_tidx(unsigned tidx) {
	auto qi = tidx_to_qitem_map.at(tidx);
	/* we have to remove multiple elements in case of a multi-edge event
	 * however, we might want to update events that already where removed,
	 * so no assertion needed here, instead we 're'-insert such an event
	 * as this should not occur to often hopefully performance does not drop */
	if(qi == NULL) {
		LOG(INFO) << "|||| re-insert idx: " << tidx;
		insert(tidx);
	} else {
		assert(NULL != qi);
		fix_idx(qi);
		tidx_in_need_update[tidx] = false;
	}
}

void
EventQueue::
insert(unsigned tidx) {
	auto qi = std::make_shared<EventQueueItem>(&events[tidx]);
	tidx_to_qitem_map_add(&events[tidx], qi);
	add_element(qi);
}

void
EventQueue::
process_pending_updates() {

	for (auto t : need_dropping) {
		assert(t);
		drop_by_tidx(t->mainEdge );
	}
	need_dropping.clear();

	for (auto t : need_update) {
		assert(t);
		update_by_tidx(t->mainEdge);
	}
	need_update.clear();

}

void
EventQueue::
assert_no_pending() const {
	assert(need_update.empty());
	assert(need_dropping.empty());
}

/** Mark a triangle as needing an update in the priority queue.
 *
 * In general, this implies its collapse spec has become invalidated,
 * however during event refinement we may actually already have set
 * the new collapse spec and it's valid, in which case we need
 * to pass bool may_have_valid_collapse_spec to appease the assertion.
 */
void
EventQueue::
needs_update(const Event * t, bool may_have_valid_collapse_spec) {
	assert(tidx_in_need_update.size() > t->mainEdge);
	if (! tidx_in_need_update[t->mainEdge]) {
		tidx_in_need_update[t->mainEdge ] = true;
		need_update.push_back(t);
		auto ubm = need_update.back();
	}

	assert(!tidx_in_need_dropping[t->mainEdge]); /* Can't drop and update both */
}

void
EventQueue::
needs_dropping(Event * t) {
	assert(tidx_in_need_dropping.size() > t->mainEdge);

	assert(!tidx_in_need_dropping[t->mainEdge]);
	tidx_in_need_dropping[t->mainEdge] = true;

	need_dropping.push_back(t);

	assert(!tidx_in_need_update[t->mainEdge]); /* Can't drop and update both */
}

bool
EventQueue::
in_needs_update(const ul edgeIdx) const {
	return tidx_in_need_update[edgeIdx];
}

bool
EventQueue::
in_needs_dropping(const Event * t) const {
	return tidx_in_need_dropping[t->mainEdge];
}

/** checks whether the heap satisfies the heap property.
 *
 * Runs in linear time, so should only be used as debugging tool.
 */
bool
EventQueue::
is_valid_heap() const {
#ifndef WITH_FP
	for (int i=size()-1; i>0; --i) {
		int parent = parent_idx(i);
		// (v1-v2).Rep()->getExactSign()
		NT delta = peak(parent)->get_priority().time() - peak(i)->get_priority().time();
		if (delta.Rep()->getSign() != delta.Rep()->getExactSign()) {
			LOG(ERROR) << "Sign mismatch at heap item " << parent << " vs. " << i;
			return false;
		}
		if (peak(parent)->get_priority() > peak(i)->get_priority()) {
			LOG(ERROR) << "Mismatch at heap item " << parent << " vs. " << i;
			return false;
		}
	}
#endif
	return Base::is_heap();
}
