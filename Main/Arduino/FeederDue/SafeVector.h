//######################################

//=========== SafeVector.h =============

//######################################

#ifndef SAFEVECTOR_H
#define SAFEVECTOR_H

// Globals to track error 
extern const int VEC_MAX_ERR = 10;
extern int VEC_CNT_ERR = 0;
extern char VEC_STR_LIST_ERR[VEC_MAX_ERR][100] = { 0 };

//============= LIBRARIES ==============

#include "Arduino.h"
//
#include <stdarg.h>

//============= CLASS: VEC =============

template<typename T>
class VEC {

private:

	// Stores array capacity
	size_t v_cap;

	// Stores actual number of stored objects
	size_t v_len;

	// Store line number
	int v_line;

	// Data pointer
	T *p_data;

public:

	// Default constructor
	VEC() : v_cap(0), v_line(0), v_len(0), p_data(0) {}

	// Constructor without data decliration
	VEC(size_t size, int line) :
		v_cap(size),
		v_line(line),
		v_len(0),
		p_data(new T[size])
	{
		for (size_t i = 0; i < size; i++)
		{
			p_data[i] = NULL;
		}
	}

	// Constructor to initialize values with arr
	VEC(size_t size, int line, const T* a_in) :
		v_cap(size),
		v_line(line),
		v_len(size),
		p_data(new T[size])
	{
		memcpy(p_data, a_in, v_cap * sizeof(T));
	}

	// Destructor
	virtual ~VEC()
	{
		delete[] p_data;
	}

	// Get capacity
	size_t cap() const
	{
		return v_cap;
	};

	// Get length
	size_t lng() const
	{
		return v_len;
	};

	// Get data
	T* data() const
	{
		return p_data;
	}

	// Opperator [] get for non-const data
	T &operator[](size_t idx)
	{
		// Check idx in range
		size_t idx_out = check_idx(idx);

		// Set lenth to max of idx or len
		if (idx_out == idx)
		{
			v_len = max(idx + 1, v_len);
		}

		// Set length to capacity
		else {
			v_len = v_cap;
		}

		// Return ref idx
		return p_data[idx_out];

	};

	// Opperator [] get for const data
	T const &operator[](size_t idx) const
	{
		// Check idx in range
		size_t idx_out = check_idx(idx);

		// Return ref idx
		return p_data[idx_out];

	};

	// Log errors
	size_t check_idx(size_t idx) const
	{
		// Use last entry ind if out of bounds
		size_t idx_out = idx < v_cap ?
			idx : v_cap - 1;

		// Log error
#if DO_VEC_DEBUG
		if (idx_out != idx)
		{

			// Get log ind
			int log_ind = min(VEC_CNT_ERR, VEC_MAX_ERR - 1);

			// Store error string
			sprintf(VEC_STR_LIST_ERR[log_ind],
				"%s: idx[%d] > cap[%d]: err_cnt=%d line_def=%d",
				log_ind != VEC_CNT_ERR ? "[*VEC_Q-DROP*]" : "VEC", idx, v_cap, VEC_CNT_ERR + 1, v_line - 13);

			// Incriment error
			VEC_CNT_ERR++;
		}
#endif
		// Return ind
		return idx_out;

	}

};



#endif