#include "EventQueue.h"
#include "CollapseSpec.h"

HeapEvent::
HeapEvent(const Event *  p_t, const NT& now)
: CollapseSpec(now)
, e(p_t)
{
}

void
HeapEvent::
update_collapse(const NT& now) {
	//  DBG_FUNC_BEGIN(DBG_EVENTQ);

	CollapseSpec::operator=( CollapseSpec(now) );

	//  DBG_FUNC_END(DBG_EVENTQ);
}

EventQueue::
EventQueue(const Events& events, Chain chain) {
	ArrayType a;
	tidx_to_qitem_map.resize(events.size(), NULL);
	tidx_in_need_dropping.resize(events.size(), false);
	tidx_in_need_update.resize(events.size(), false);

	/* we skip the first and last edge of each chain */
	for (auto t = std::next(chain.begin()); t != std::prev(chain.end()); ++t) {
		LOG(INFO) << "add event " << events[*t];
		auto qi = std::make_shared<EventQueueItem>(&events[*t], events[*t].eventTime);
		a.emplace_back(qi);
		tidx_to_qitem_map_add(&events[*t], qi);
	}
	setArray(a);
	//  DBG(DBG_EVENTQ) << "heap array:";
	//  for (int i=0; i < size(); ++i) {
	//    DBG(DBG_EVENTQ) << " item " << i << ": " << peak(i)->get_priority();
	//  }
	//	const auto x = peak();
	//  DBG(DBG_EVENTQ) << "top: " << x->get_priority();
	//#if defined (DEBUG_EXPENSIVE_PREDICATES) && DEBUG_EXPENSIVE_PREDICATES >= 1
	//  if (! is_valid_heap() ) {
	//    LOG(ERROR) << "Heap is not valid!";
	//    exit(EXIT_INIT_INVALID_HEAP);
	//  }
	//#endif
}

void
EventQueue::
tidx_to_qitem_map_add(const Event * t, ElementType qi) {
	unsigned id = t->mainEdge;
	/*
  if (id >= tidx_to_qitem_map.size()) {
    tidx_to_qitem_map.resize(id+1);
  };
	 */
	//  if(id >= tidx_to_qitem_map.size()) {LOG(INFO) << "oh .. " << *t << " with id: "<< id
	//	  << " for map size: " << tidx_to_qitem_map.size();}
	assert(id < tidx_to_qitem_map.size());
	tidx_to_qitem_map[id] = qi;
}

void
EventQueue::
drop_by_tidx(unsigned tidx) {
	// DBG_FUNC_BEGIN(DBG_EVENTQ);
	//  DBG(DBG_EVENTQ) << "tidx: " << tidx;

	assert(tidx_in_need_dropping[tidx]);
	auto qi = tidx_to_qitem_map.at(tidx);
	assert(NULL != qi);
	drop_element(qi);
	tidx_to_qitem_map[tidx] = NULL;
	tidx_in_need_dropping[tidx] = false;

	// DBG_FUNC_END(DBG_EVENTQ);
}

void
EventQueue::
update_by_tidx(unsigned tidx, const NT& now) {
	// DBG_FUNC_BEGIN(DBG_EVENTQ);
	//  DBG(DBG_EVENTQ) << "tidx: " << tidx << "; now: " << CGAL::to_double(now);

	assert(tidx_in_need_update[tidx]);
	auto qi = tidx_to_qitem_map.at(tidx);
	assert(NULL != qi);
	qi->update_priority(now);
	fix_idx(qi);
	tidx_in_need_update[tidx] = false;

	// DBG_FUNC_END(DBG_EVENTQ);
}


void
EventQueue::
process_pending_updates(const NT& now) {

	for (auto t : need_dropping) {
		assert(t);
		drop_by_tidx(t->mainEdge );
	}
	need_dropping.clear();

	LOG(INFO) << "update size: " << need_update.size();
	for (auto t : need_update) {
		assert(t);
		LOG(INFO) << "update: " << t->mainEdge;
		update_by_tidx(t->mainEdge, now);
	}
	need_update.clear();

	LOG(INFO) << "update size: " << need_update.size(); fflush(stdout);
	LOG(INFO) << "dropping size: " << need_dropping.size(); fflush(stdout);
	assert_no_pending();
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
	// DBG_FUNC_BEGIN(DBG_EVENTQ);
	LOG(INFO) << "t " << *t;

	assert(tidx_in_need_update.size() > t->mainEdge);
	//  assert(!t->is_collapse_spec_valid() || may_have_valid_collapse_spec);
	// during refinement, the same triangle may be tagged as needs_update multiple times.
	if (! tidx_in_need_update[t->mainEdge]) {
		tidx_in_need_update[t->mainEdge ] = true;
		need_update.push_back(t);

		LOG(INFO) << "added " << t->mainEdge;
		auto ubm = need_update.back();
		LOG(INFO) << "aka " << ubm->mainEdge;
	}

	assert(!tidx_in_need_dropping[t->mainEdge]); /* Can't drop and update both */
	// DBG_FUNC_END(DBG_EVENTQ);
}

void
EventQueue::
needs_dropping(Event * t) {
	// DBG_FUNC_BEGIN(DBG_EVENTQ);
	LOG(INFO) << "t" << *t;

	assert(tidx_in_need_dropping.size() > t->mainEdge);
	//  assert(t->is_dying());
	//  t->set_dead();

	assert(!tidx_in_need_dropping[t->mainEdge]);
	tidx_in_need_dropping[t->mainEdge] = true;

	need_dropping.push_back(t);

	assert(!tidx_in_need_update[t->mainEdge]); /* Can't drop and update both */
	// DBG_FUNC_END(DBG_EVENTQ);
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
#ifndef NT_USE_DOUBLE
	for (int i=size()-1; i>0; --i) {
		int parent = parent_idx(i);
		// (v1-v2).Rep()->getExactSign()
		NT delta = peak(parent)->get_priority().time() - peak(i)->get_priority().time();
		if (delta.Rep()->getSign() != delta.Rep()->getExactSign()) {
			LOG(ERROR) << "Sign mismatch at heap item " << parent << " vs. " << i;
			//      DBG(DBG_EVENTQ) << " item " << parent << ": " << peak(i)->get_priority();
			//      DBG(DBG_EVENTQ) << " item " << i << ": " << peak(parent)->get_priority();
			//      DBG(DBG_EVENTQ) << " delta is " << delta;
			//      DBG(DBG_EVENTQ) << " sign is " << delta.Rep()->getSign();
			//      DBG(DBG_EVENTQ) << " exact sign is " << delta.Rep()->getExactSign();
			return false;
		}
		if (peak(parent)->get_priority() > peak(i)->get_priority()) {
			LOG(ERROR) << "Mismatch at heap item " << parent << " vs. " << i;
			//      DBG(DBG_EVENTQ) << " item " << parent << ": " << peak(i)->get_priority();
			//      DBG(DBG_EVENTQ) << " item " << i << ": " << peak(parent)->get_priority();
			return false;
		}
	}
#endif
	return Base::is_heap();
}

//std::ostream&
//operator<<(std::ostream& os, const Event& e) {
////  os << "Event in " << e.mainEdge << " " << CollapseSpec(e);
//  return os;
//}

