#ifndef __array_
#define __array_

/*
	_array Template Class

2007. Sep. 30.
	method :
		bool contains(const TYPE &value);
		TYPE& get_at(int i);

2006. Dec. 19.
	Add Clear Memory method		Jaeyoung Han



2003.Sep.2.
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr

	template <class TYPE> class _array
	method : 
		_array();
		_array(int sz);
		_array(const _array<TYPE>&ary);
		~_array();
		int get_size(void) const;
		void set_size(int n);
		const TYPE &operator [] (int i) const;
		TYPE &operator [] (int i);
		_array &operator = (const _array<TYPE>&ary);
		int find(const TYPE &value) const;
		int find_next(const TYPE &value, int idx) const	;
		void check_add_tail(const TYPE &value);
		void add_tail(const TYPE &value);
		void add_tail(const _array<TYPE>&ary);
		void add_head(const TYPE &value);
		void pop(int idx);
*/


#include <assert.h>
#include <memory>

#ifndef NULL
#define NULL 0
#endif

/*!
	\class _array
	\brief a simple port of STL vector template
	
	_array is a template class to facilitate managing data in an array.
*/
template<class TYPE>
class _array {
public :
    _array() {
        size = mem_size = 0;
    }

    /*!
        constructor
        \param sz a number of item slots in the array
    */
    _array(int sz) {
        assert(sz >= 0 && "_array(int) -> invalid size");

        mem_size = (size = sz) << 2;
        if (mem_size > 0) base = std::unique_ptr<TYPE[]>(new TYPE[mem_size]);
    }

    /*!
        copy constructor
    */
    _array(const _array<TYPE> &ary) {
        base = std::unique_ptr<TYPE[]>(new TYPE[mem_size = (size = ary.size) << 2]);
        int n = size;
        TYPE *base_i = base.get(), *ary_base_i = ary.base.get();
        while (n--) *base_i++ = *ary_base_i++;
    }

    ~_array() = default;

    /*!
        \return a number of items in the array
    */
    int get_size(void) const {
        return size;
    }

    /*!
        adjust a size of the array.
        \param n a number of item slot in the array
        \param keep_value In order to keep values in the array after adjusting a size, set 'keep_value' to be true.
        If 'keep_value' is false, values in the array after adjunsting a size may change.
    */
    void set_size(int n, bool keep_value = false) {
        assert(n >= 0 && "_array::set_size(int, bool) -> invalid size");

        if (n > mem_size) {
            mem_size = n << 2;
            auto tmp = std::unique_ptr<TYPE[]>(new TYPE[mem_size]);
            if (keep_value) {
                int n = size;
                TYPE *tmp_i = tmp.get(), *base_i = base.get();
                while (n--) *tmp_i++ = *base_i++;
            }
            base = std::move(tmp);
        }
        size = n;
    }

    const TYPE &operator[](int i) const {
        assert(i >= 0 && i < size && "_array::operator[int] -> invalid index");
        return base[i];
    }

    /*!
        access to the i-th item in the array.
    */
    TYPE &operator[](int i) {
        assert(i >= 0 && i < size && "_array::operator[int] -> invalid index");
        return base[i];
    }

    /*!
        equal operator
    */
    _array<TYPE> &operator=(const _array<TYPE> &ary) {
        if (ary.size > mem_size) {
            base = std::unique_ptr<TYPE[]>(new TYPE[mem_size = ary.size << 2]);
        }
        size = ary.size;

        int n = size;
        TYPE *base_i = base.get(), *ary_base_i = ary.base.get();
        while (n--) *base_i++ = *ary_base_i++;

        for (int i = 0; i < size; i++) base[i] = ary.base[i];
        return *this;
    }

    /*!
        find value in the array.
        \return index count where value is found.
        If there is no item equal to value, return -1.
    */
    int find(const TYPE &value) const {
        int i = 0, n = size;
        TYPE *base_i = base.get();
        while (n--) {
            if (*base_i++ == value) return i;
            i++;
        }
        return -1;
    }

    /*!
        find a value in the array after the idx-th item.
    */
    int find_next(const TYPE &value, int idx) const {
        int i = idx + 1, n = size - idx - 1;
        TYPE *base_i = base.get() + i;
        while (n--) {
            if (*base_i++ == value) return i;
            i++;
        }
        return -1;
    }

    /*!
        add an item at tail.
    */
    void add_tail(const TYPE &value) {
        int n = size + 1;
        if (n > mem_size) {
            mem_size = n << 2;
            auto tmp = std::unique_ptr<TYPE[]>(new TYPE[mem_size]);
            TYPE *tmp_i = tmp.get(), *base_i = base.get();
            while (--n) *tmp_i++ = *base_i++;
            base = std::move(tmp);
        }
        base[size++] = value;
    }

    /*!
        if 'value' is in the array, else add it at tail.
    */
    void check_add_tail(const TYPE &value) {
        int n = size;
        TYPE *base_i = base.get();
        while (n--) if (*base_i++ == value) return;

        n = size + 1;
        if (n > mem_size) {
            mem_size = n << 2;
            auto tmp = std::unique_ptr<TYPE[]>(new TYPE[mem_size]);
            TYPE *tmp_i = tmp.get(), *base_i = base.get();
            while (--n) *tmp_i++ = *base_i++;
            base = std::move(tmp);
        }
        base[size++] = value;
    }

    /*!
        add another array at tail.
    */
    void add_tail(const _array<TYPE> &ary) {
        if (size + ary.size > mem_size) {
            int n = size;
            auto tmp = std::unique_ptr<TYPE[]>(new TYPE[mem_size = (size + ary.size) << 2]);
            TYPE *tmp_i = tmp.get(), *base_i = base.get();
            while (n--) *tmp_i++ = *base_i++;
            base = std::move(tmp);
        }

        int n = ary.size;
        TYPE *base_i = base.get() + size, *ary_base_i = ary.base.get();
        size += n;
        while (n--) *base_i++ = *ary_base_i++;
    }

    /*!
        add an item at head.
    */
    void add_head(const TYPE &value) {
        int n = size + 1;
        if (n > mem_size) {
            mem_size = n << 2;
            auto tmp = std::unique_ptr<TYPE[]>(new TYPE[mem_size]);
            TYPE *tmp_i = tmp.get() + 1, *base_i = base.get();
            while (--n) *tmp_i++ = *base_i++;
            base = std::move(tmp);
        } else {
            TYPE *base_i = base.get() + size, *base_im1 = base_i - 1;
            while (--n) *base_i-- = *base_im1--;
        }
        size++;
        base[0] = value;
    }

    /*!
        pop out the idx-th item.
    */
    void pop(int idx) {
        if (idx < 0 || idx >= size) return;

        int n = size - idx;
        TYPE *base_i = base.get() + idx, *base_ip1 = base_i + 1;
        while (--n) *base_i++ = *base_ip1++;
        size--;
    }

    /*!
        Clear all items.	'06. Dec. 19, Jaeyoung Han
    */
    void clear(void) {
        base.reset();

        size = mem_size = 0;
    }

    void remove(const TYPE &value) {
        pop(find(value));
    }

    /*!
    Find and pop item.	'07. September. 28, Jeongseok Lee
    */
    void find_pop(const TYPE &value) {
        int i = 0, n = size;
        TYPE *base_i = base.get();
        while (n--) {
            if (*base_i++ == value) {
                n = size - i;
                base_i = base.get() + i;
                TYPE *base_ip1 = base_i + 1;
                while (--n)
                    *base_i++ = *base_ip1++;
                size--;
            }
            i++;
        }
    }

    /*!
        whether contains item.	'07. September. 28, Jeongseok Lee
    */
    bool contains(const TYPE &value) {
        int i = 0, n = size;
        TYPE *base_i = base.get();
        while (n--) {
            if (*base_i++ == value) return true;
            i++;
        }
        return false;
    }

    /*!
    '07. September. 28, Jeongseok Lee
    access to the i-th item in the array.
    */
    TYPE &get_at(int i) {
        assert(i >= 0 && i < size && "_array::operator[int] -> invalid index");
        return base[i];
    }

private :

    int size;
    int mem_size;
    std::unique_ptr<TYPE[]> base;
};

#endif
