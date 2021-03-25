#ifndef PLUGGABLE_PHYSICS_RID_H
#define PLUGGABLE_PHYSICS_RID_H

#include "server.h"
#include "core/list.h"
#include "core/os/memory.h"
#include "core/typedefs.h"

/* This is a class that has the _exact_ same memory layout of the default RID
 * class but doesn't force inefficient use of a Set.
 * It is still not as efficient as it could be due to the needing to match the
 * layout but oh well
 *
 * The way this class works is simple:
 * - Receive a 32 bit integer and store it in FastRID_Data as an integer
 * - Store the FastRID in a big ol' array at the `_id` location
 * - Wrap a pointer inside a RID
 *
 * Note that a custom array type should be used because Vector's COW nature
 * makes it inefficient. A custom array type is defined below.
 *
 * This entire file is terrible and there is a very good reason I force myself to write safe Rust.
 *
 * I will nuke these classes at some point, I promise
 */

class FastRID_Array;

class FastRID_Data {
	// I have no idea what is up with this. Why is this field removed
	// in debug mode? Why not just keep it?
	// In any case, it effectively limits us to only 2^32 ids, which should
	// still be plenty for any game that runs at more than 1 FPS.
	// Do note that this field still needs to be zeroed in case a RID is passed
	// to the wrong server / RID set
#ifndef DEBUG_ENABLED
	void *_owner;
#endif
	// Public for ease of access
public:
	FastRID_Data() {
#ifndef DEBUG_ENABLED
		this->_owner = nullptr;
#endif
		this->id = 0xffffffff;
	}

	uint32_t id;

	_FORCE_INLINE_ uint32_t get_id() const { return id; }

	// Shame, this makes the class quite a bit larger. It also makes it impossible
	// to use some (dangerous) array trickery to secretly pass around 32 bit integers
	// and pray nothing would access the owner field somehow.
	virtual ~FastRID_Data() {}
};

class FastRID {
	mutable FastRID_Data *_data;

public:
	// This derefence forces the use of heap memory
	_FORCE_INLINE_ uint32_t get_id() const { return _data ? _data->get_id() : 0; }

	_FORCE_INLINE_ FastRID(FastRID_Data *p_data) {
		_data = p_data;
	}
};

class FastRID_Array {
	// This is indeed an array of pointers to FastRID_Datas
	// This is because reallocs are a thing sadly
	// The impact could be lessened (and cache coherency increased) by using
	// "chunks" but keep it simple for now
	FastRID_Data **pointer;
	// A unsigned 32 bit integer is fine, as IDs are limited to 32 bits anyways
	uint32_t length;
	// A capacity field isn't strictly necessary for our purposes, nor does
	// length need to be exposed. So the length is used as a capacity field instead
	// It will be useful if we ever want to shrink the array in the future though.
	uint32_t iterator_index = 0;

public:
	FastRID_Array() {
		// Reserve the first RID as 0 == invalid
		this->length = 1;
		this->pointer = (FastRID_Data **)memalloc(this->length * sizeof(FastRID_Data *));
		this->pointer[0] = nullptr;
	}

	~FastRID_Array() {
		memfree(this->pointer);
	}

	// Returns a RID with _data pointing to an element inside the array. _data will be
	// null if the ID isn't valid or the ID is out of bounds
	RID get(uint32_t id) {
		ERR_FAIL_INDEX_V_MSG(id, this->length, RID(), "ID is out of range");
		FastRID rid(this->pointer[id]);
		// SAFETY: the memory layout of FastRID_Data and FastRID is the same
		// as that of RID_Data and RID. Both have empty destructors.
		RID real_rid;
		static_assert(sizeof(rid) == sizeof(real_rid));
		memcpy((void *)&real_rid, &rid, sizeof(rid));
		return real_rid;
	}

	// Create a new FastRID_Data with the given id and return a wrapper RID
	// Note that this will leak if the given ID wasn't deallocated yet
	RID create(uint32_t id) {
		if (id == 0) {
			// Just ignore for now, as server_gen.cpp doesn't check for id == 0
			return RID();
		}
		if (id >= this->length) {
			// Memory::realloc_static has a pad argument which is false by default,
			// but it uses realloc internally which should already be aligning anyways.
			uint64_t new_length_64 = (this->length + 1) * 3 / 2;
			uint32_t new_length = new_length_64 > 0xffffffff ? 0xffffffff : (uint32_t)new_length_64;
			FastRID_Data **temp = (FastRID_Data **)memrealloc(this->pointer, new_length * sizeof(FastRID_Data *));
			ERR_FAIL_COND_V_MSG(temp == nullptr, RID(), "Failed to reallocate array");
			this->pointer = temp;
			this->length = new_length;
		}
		this->pointer[id] = memnew(FastRID_Data);
		this->pointer[id]->id = id;
		return this->get(id);
	}

	// Remove an existing FastRID_Data. Doesn't do anything if the ID is out of range
	void remove(uint32_t id) {
		ERR_FAIL_INDEX_MSG(id, this->length, "ID is out of range");
		if (id < this->length) {
			// A null check is not needed as free() ignores nulls anyways.
			memfree(this->pointer[id]);
			this->pointer[id] = nullptr;
		}
	}

	// Resets the iterator and returns the first valid ID or 0 if no valid IDs are found
	uint32_t iter() {
		this->iterator_index = 0;
		return this->next();
	}

	// Returns the next valid ID or 0 if no valid IDs are found
	uint32_t next() {
		for ( ; this->pointer[this->iterator_index] == 0; this->iterator_index++) {
			if (this->iterator_index >= this->length) {
				return 0;
			}
		}
		return this->iterator_index++;
	}
};


#endif
