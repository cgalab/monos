#pragma once

#include <stddef.h>

#include "cgTypes.h"
#include "CollapseSpec.h"
#include "Heap.h"

class HeapEvent : public CollapseSpec {
public:
	const Event * const e;
	HeapEvent(const Event * p_t, const NT& now);

	void update_collapse(const NT& t);

	friend std::ostream& operator<<(std::ostream& os, const Event& e);
};

class EventQueueItem : public HeapItemBase <HeapEvent> {
private:
public:
	EventQueueItem(const Event * e, const NT& now)
: HeapItemBase<HeapEvent>(HeapEvent(e, now))
  {};

	void update_priority(const NT& now) {
		priority.update_collapse(now);
	}
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
	void drop_by_tidx(unsigned tidx);
	void assert_no_pending() const;
public:
	EventQueue(const Events& events, Chain chain);

	void update_by_tidx(unsigned tidx, const NT& now);
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

	void process_pending_updates(const NT& now);

	void needs_update(const Event * t, bool may_have_valid_collapse_spec = false);
	void needs_dropping(Event * t);

	bool in_needs_update(const ul edgeIdx) const;
	bool in_needs_dropping(const Event * t) const;

	bool is_valid_heap() const;
};
