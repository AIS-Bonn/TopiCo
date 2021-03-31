/* Copyright 2019 The Mathworks, Inc. */
/* Copied from fullfile(matlabroot,'extern','include','coder','coder_array','coder_array_rtw.h') */

#ifndef _mw_coder_array_h
#define _mw_coder_array_h

//  Usage:
//
//  coder::array<T, N>: T base type of data, N number of dimensions
//
//  coder::array()
//               : default constructor
//  coder::array(const coder::array &)
//               : copy constructor (always make a deep copy of other array)
//  coder::array(const T *data, const SizeType *sz)
//               : Set data with sizes of this array.
//               : (Data is not copied, data is not deleted)
//  coder::array::operator = (coder coder::array &)
//               : Assign into this array;
//               : delete its previous contents (if owning the data.)
//  set(const T *data, SizeType sz1, SizeType sz2, ...)
//               : Set data with dimensions.
//               : (Data is not copied, data is not deleted)
//  set_size(SizeType sz1, SizeType sz2, ...)
//               : Set sizes of array. Reallocate memory of data if needed.
//  bool is_owner() : Return true if the data is owned by the class.
//  void set_owner(b) : Set if the data is owned by the class.
//  SizeType capacity() : How many entries are reserved by memory allocation.
//  reshape( SizeType sz1, SizeType sz2, ...)
//               : Reshape array to a different ND shape. Do not copy the data.
//               : The number of elements must not be changed (numel()==sz1*sz2*...)
//               : Return the array with possibly new number of dimensions.
//  clear()      : Reset array to be empty.
//  SizeType numel() : Return the number of elements.
//  operator [] (SizeType index) : Extract element at linear index (0 based.)
//  size(SizeType dimension) : Size of array of the provided dimension.
//  SizeType * size() : Return the pointer to all the sizes of this array.
//  SizeType index(SizeType i1, SizeType i2, ...)
//               : Compute the linear index from ND index (i1,i2,...)
//  at(SizeType i1, SizeType i2, ...) : The element at index (i1,i2,...)

#include <algorithm>
#include <cstring>
#include <iterator>
#include <string>
#include <vector>

#ifndef INT32_T
#include "rtwtypes.h"
#endif

namespace coder {

#ifndef CODER_ARRAY_NEW_DELETE
#define CODER_ARRAY_NEW_DELETE
#define CODER_NEW(T, N) new T[N]
#define CODER_DELETE(P) delete[](P)
#endif

typedef int32_T SizeType;
namespace std = ::std;

namespace detail {

#ifndef CODER_ARRAY_DATA_PTR_DEFINED
template <typename T, typename SZ>
class data_ptr {
  public:
    typedef T value_type;
    typedef SZ size_type;

    data_ptr()
        : data_(NULL)
        , size_(0)
        , capacity_(0)
        , owner_(false) {
    }
    data_ptr(T* _data, SZ _sz)
        : data_(_data)
        , size_(_sz)
        , capacity_(_sz)
        , owner_(false) {
    }

    data_ptr(const data_ptr& _other)
        : data_(_other.owner_ ? NULL : _other.data_)
        , size_(_other.owner_ ? 0 : _other.size_)
        , capacity_(_other.owner_ ? 0 : _other.capacity_)
        , owner_(_other.owner_) {
        if (owner_) {
            resize(_other.size_);
            (void)std::copy(_other.data_, _other.data_ + size_, data_);
        }
    }

    ~data_ptr() {
        if (owner_) {
            CODER_DELETE(data_);
        }
    }
    SZ capacity() const {
        return capacity_;
    }
    void reserve(SZ _n) {
        if (_n > capacity_) {
            T* new_data = CODER_NEW(T, _n);
            (void)std::copy(data_, data_ + size_, new_data);
            if (owner_) {
                CODER_DELETE(data_);
            }
            data_ = new_data;
            capacity_ = _n;
            owner_ = true;
        }
    }
    void resize(SZ _n) {
        reserve(_n);
        size_ = _n;
    }

  private:
    // Prohibit use of assignment operator to prevent subtle bugs
    void operator=(const data_ptr<T, SZ>& _other);

  public:
    void set(T* _data, const SZ _sz) {
        if (owner_) {
            CODER_DELETE(data_);
        }
        data_ = _data;
        size_ = _sz;
        owner_ = false;
        capacity_ = size_;
    }

    void copy(const T* _data, SZ _size) {
        if (data_ == _data) {
            size_ = _size;
            return;
        }
        if (owner_) {
            CODER_DELETE(data_);
        }
        data_ = CODER_NEW(T, _size);
        owner_ = true;
        size_ = _size;
        capacity_ = size_;
        (void)std::copy(_data, _data + _size, data_);
    }

    void copy(const data_ptr<T, SZ>& _other) {
        copy(_other.data_, _other.size_);
    }

    operator T*() {
        return &data_[0];
    }

    operator const T*() const {
        return &data_[0];
    }

    T& operator[](SZ _index) {
        return data_[_index];
    }
    const T& operator[](SZ _index) const {
        return data_[_index];
    }

    T* operator->() {
        return data_;
    }

    const T* operator->() const {
        return data_;
    }

    bool is_null() const {
        return data_ == NULL;
    }

    void clear() {
        if (owner_) {
            CODER_DELETE(data_);
        }
        data_ = NULL;
        size_ = 0;
        capacity_ = 0;
        owner_ = false;
    }

    bool is_owner() const {
        return owner_;
    }

    void set_owner(bool _b) {
        owner_ = _b;
    }

  private:
    T* data_;
    SZ size_;
    SZ capacity_;
    bool owner_;
};
#endif

} // namespace detail

// Implementing the random access iterator class so coder::array can be
// used in STL iterators.
template <typename T>
class array_iterator : public std::iterator<std::random_access_iterator_tag,
                                            typename T::value_type,
                                            typename T::size_type> {
  public:
    array_iterator()
        : arr_(NULL)
        , i_(0) {
    }
    array_iterator(const array_iterator<T>& other)
        : arr_(other.arr_)
        , i_(other.i_) {
    }
    ~array_iterator() {
    }
    typename T::value_type& operator*() const {
        return (*arr_)[i_];
    }
    typename T::value_type* operator->() const {
        return &(*arr_)[i_];
    }
    typename T::value_type& operator[](typename T::size_type _di) const {
        return (*arr_)[i_ + _di];
    }
    array_iterator<T>& operator++() {
        ++i_;
        return *this;
    }
    array_iterator<T>& operator--() {
        --i_;
        return *this;
    }
    array_iterator<T> operator++(int) {
        array_iterator<T> cp(*this);
        ++i_;
        return cp;
    }
    array_iterator<T> operator--(int) {
        array_iterator<T> cp(*this);
        --i_;
        return cp;
    }
    array_iterator<T>& operator=(const array_iterator<T>& _other) {
        this->i_ = _other.i_;
        return *this;
    }
    bool operator==(const array_iterator<T>& _other) const {
        return i_ == _other.i_;
    }
    bool operator!=(const array_iterator<T>& _other) const {
        return i_ != _other.i_;
    }
    bool operator<(const array_iterator<T>& _other) const {
        return i_ < _other.i_;
    }
    bool operator>(const array_iterator<T>& _other) const {
        return i_ > _other.i_;
    }
    bool operator<=(const array_iterator<T>& _other) const {
        return i_ <= _other.i_;
    }
    bool operator>=(const array_iterator<T>& _other) const {
        return i_ >= _other.i_;
    }
    array_iterator<T> operator+(typename T::size_type _add) const {
        array_iterator<T> cp(*this);
        cp.i_ += _add;
        return cp;
    }
    array_iterator<T>& operator+=(typename T::size_type _add) {
        this->i_ += _add;
        return *this;
    }
    array_iterator<T> operator-(typename T::size_type _subtract) const {
        array_iterator<T> cp(*this);
        cp.i_ -= _subtract;
        return cp;
    }
    array_iterator<T>& operator-=(typename T::size_type _subtract) {
        this->i_ -= _subtract;
        return *this;
    }
    typename T::size_type operator-(const array_iterator<T>& _other) const {
        return static_cast<typename T::size_type>(this->i_ - _other.i_);
    }

    array_iterator(T* _arr, typename T::size_type _i)
        : arr_(_arr)
        , i_(_i) {
    }

  private:
    T* arr_;
    typename T::size_type i_;
};

// Const version of the array iterator.
template <typename T>
class const_array_iterator : public std::iterator<std::random_access_iterator_tag,
                                                  typename T::value_type,
                                                  typename T::size_type> {
  public:
    const_array_iterator()
        : arr_(NULL)
        , i_(0) {
    }
    const_array_iterator(const const_array_iterator<T>& other)
        : arr_(other.arr_)
        , i_(other.i_) {
    }
    ~const_array_iterator() {
    }
    const typename T::value_type& operator*() const {
        return (*arr_)[i_];
    }
    const typename T::value_type* operator->() const {
        return &(*arr_)[i_];
    }
    const typename T::value_type& operator[](typename T::size_type _di) const {
        return (*arr_)[i_ + _di];
    }
    const_array_iterator<T>& operator++() {
        ++i_;
        return *this;
    }
    const_array_iterator<T>& operator--() {
        --i_;
        return *this;
    }
    const_array_iterator<T> operator++(int) {
        const_array_iterator<T> copy(*this);
        ++i_;
        return copy;
    }
    const_array_iterator<T> operator--(int) {
        const_array_iterator copy(*this);
        --i_;
        return copy;
    }
    const_array_iterator<T>& operator=(const const_array_iterator<T>& _other) {
        this->i_ = _other.i_;
        return *this;
    }
    bool operator==(const const_array_iterator<T>& _other) const {
        return i_ == _other.i_;
    }
    bool operator!=(const const_array_iterator<T>& _other) const {
        return i_ != _other.i_;
    }
    bool operator<(const const_array_iterator<T>& _other) const {
        return i_ < _other.i_;
    }
    bool operator>(const const_array_iterator<T>& _other) const {
        return i_ > _other.i_;
    }
    bool operator<=(const const_array_iterator<T>& _other) const {
        return i_ <= _other.i_;
    }
    bool operator>=(const const_array_iterator<T>& _other) const {
        return i_ >= _other.i_;
    }
    const_array_iterator<T> operator+(typename T::size_type _add) const {
        const_array_iterator<T> cp(*this);
        cp.i_ += _add;
        return cp;
    }
    const_array_iterator<T>& operator+=(typename T::size_type _add) {
        this->i_ += _add;
        return *this;
    }
    const_array_iterator<T> operator-(typename T::size_type _subtract) const {
        const_array_iterator<T> cp(*this);
        cp.i_ -= _subtract;
        return cp;
    }

    const_array_iterator<T>& operator-=(typename T::size_type _subtract) {
        this->i_ -= _subtract;
        return *this;
    }

    typename T::size_type operator-(const const_array_iterator<T>& _other) const {
        return static_cast<typename T::size_type>(this->i_ - _other.i_);
    }

    const_array_iterator(const T* _arr, typename T::size_type _i)
        : arr_(_arr)
        , i_(_i) {
    }

  private:
    const T* arr_;
    typename T::size_type i_;
    typename T::size_type n_;
};

namespace detail {

// detail::numel<N>: Compile-time product of the given size vector of length N.
template <int N>
class numel {
  public:
    template <typename SZ>
    static SZ compute(SZ _size[]) {
        return _size[N - 1] * numel<N - 1>::compute(_size);
    }
};
template <>
class numel<0> {
  public:
    template <typename SZ>
    static SZ compute(SZ[]) {
        return 1;
    }
};

// Compute flat index from (column-major) ND size vector and a list of indices.
template <int I>
class index_nd {
  public:
    template <typename SZ>
    static SZ compute(const SZ _size[], const SZ _indices[]) {
        const SZ weight = numel<I - 1>::compute(_size);
        return weight * _indices[I - 1] + index_nd<I - 1>::compute(_size, _indices);
    }
};

template <>
class index_nd<0> {
  public:
    template <typename SZ>
    static SZ compute(SZ[], SZ[]) {
        return 0;
    }
};

template <bool Cond>
struct match_dimensions {};

template <>
struct match_dimensions<true> {
    static void check() {
    }
};

} // namespace detail

// Base class for code::array. SZ is the type used for sizes (currently int32_t.)
// Overloading up to 10 dimensions (not using variadic templates to
// stay compatible with C++98.)
template <typename T, typename SZ, int N>
class array_base {
  public:
    typedef T value_type;
    typedef SZ size_type;

    array_base() {
        (void)::memset(size_, 0, sizeof(SZ) * N);
    }

    array_base(T* _data, const SZ* _sz)
        : data_(_data, coder::detail::numel<N>::compute(_sz)) {
        (void)std::copy(_sz, _sz + N, size_);
    }

    array_base& operator=(const array_base& _other) {
        data_.copy(_other.data_);
        (void)std::copy(_other.size_, _other.size_ + N, size_);
        return *this;
    }

    void set(T* _data, SZ _n1) {
        coder::detail::match_dimensions<N == 1>::check();
        data_.set(_data, _n1);
        size_[0] = _n1;
    }

    void set(T* _data, SZ _n1, SZ _n2) {
        coder::detail::match_dimensions<N == 2>::check();
        data_.set(_data, _n1 * _n2);
        size_[0] = _n1;
        size_[1] = _n2;
    }

    void set(T* _data, SZ _n1, SZ _n2, SZ _n3) {
        coder::detail::match_dimensions<N == 3>::check();
        data_.set(_data, _n1 * _n2 * _n3);
        size_[0] = _n1;
        size_[1] = _n2;
        size_[2] = _n3;
    }

    void set(T* _data, SZ _n1, SZ _n2, SZ _n3, SZ _n4) {
        coder::detail::match_dimensions<N == 4>::check();
        data_.set(_data, _n1 * _n2 * _n3 * _n4);
        size_[0] = _n1;
        size_[1] = _n2;
        size_[2] = _n3;
        size_[3] = _n4;
    }

    void set(T* _data, SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5) {
        coder::detail::match_dimensions<N == 5>::check();
        data_.set(_data, _n1 * _n2 * _n3 * _n4 * _n5);
        size_[0] = _n1;
        size_[1] = _n2;
        size_[2] = _n3;
        size_[3] = _n4;
        size_[4] = _n5;
    }

    void set(T* _data, SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5, SZ _n6) {
        coder::detail::match_dimensions<N == 6>::check();
        data_.set(_data, _n1 * _n2 * _n3 * _n4 * _n5 * _n6);
        size_[0] = _n1;
        size_[1] = _n2;
        size_[2] = _n3;
        size_[3] = _n4;
        size_[4] = _n5;
        size_[5] = _n6;
    }

    void set(T* _data, SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5, SZ _n6, SZ _n7) {
        coder::detail::match_dimensions<N == 7>::check();
        data_.set(_data, _n1 * _n2 * _n3 * _n4 * _n5 * _n6 * _n7);
        size_[0] = _n1;
        size_[1] = _n2;
        size_[2] = _n3;
        size_[3] = _n4;
        size_[4] = _n5;
        size_[5] = _n6;
        size_[6] = _n7;
    }

    void set(T* _data, SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5, SZ _n6, SZ _n7, SZ _n8) {
        coder::detail::match_dimensions<N == 8>::check();
        data_.set(_data, _n1 * _n2 * _n3 * _n4 * _n5 * _n6 * _n7 * _n8);
        size_[0] = _n1;
        size_[1] = _n2;
        size_[2] = _n3;
        size_[3] = _n4;
        size_[4] = _n5;
        size_[5] = _n6;
        size_[6] = _n7;
        size_[7] = _n8;
    }

    void set(T* _data, SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5, SZ _n6, SZ _n7, SZ _n8, SZ _n9) {
        coder::detail::match_dimensions<N == 9>::check();
        data_.set(_data, _n1 * _n2 * _n3 * _n4 * _n5 * _n6 * _n7 * _n8 * _n9);
        size_[0] = _n1;
        size_[1] = _n2;
        size_[2] = _n3;
        size_[3] = _n4;
        size_[4] = _n5;
        size_[5] = _n6;
        size_[6] = _n7;
        size_[7] = _n8;
        size_[8] = _n9;
    }

    void
    set(T* _data, SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5, SZ _n6, SZ _n7, SZ _n8, SZ _n9, SZ _n10) {
        coder::detail::match_dimensions<N == 10>::check();
        data_.set(_data, _n1 * _n2 * _n3 * _n4 * _n5 * _n6 * _n7 * _n8 * _n9 * _n10);
        size_[0] = _n1;
        size_[1] = _n2;
        size_[2] = _n3;
        size_[3] = _n4;
        size_[4] = _n5;
        size_[5] = _n6;
        size_[6] = _n7;
        size_[7] = _n8;
        size_[8] = _n9;
        size_[9] = _n10;
    }

    bool is_owner() const {
        return data_.is_owner();
    }

    void set_owner(bool b) {
        data_.set_owner(b);
    }

    SZ capacity() const {
        return data_.capacity();
    }

    void set_size(SZ _n1) {
        coder::detail::match_dimensions<N == 1>::check();
        size_[0] = _n1;
        ensureCapacity(numel());
    }

    void set_size(SZ _n1, SZ _n2) {
        coder::detail::match_dimensions<N == 2>::check();
        size_[0] = _n1;
        size_[1] = _n2;
        ensureCapacity(numel());
    }

    void set_size(SZ _n1, SZ _n2, SZ _n3) {
        coder::detail::match_dimensions<N == 3>::check();
        size_[0] = _n1;
        size_[1] = _n2;
        size_[2] = _n3;
        ensureCapacity(numel());
    }

    void set_size(SZ _n1, SZ _n2, SZ _n3, SZ _n4) {
        coder::detail::match_dimensions<N == 4>::check();
        size_[0] = _n1;
        size_[1] = _n2;
        size_[2] = _n3;
        size_[3] = _n4;
        ensureCapacity(numel());
    }

    void set_size(SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5) {
        coder::detail::match_dimensions<N == 5>::check();
        size_[0] = _n1;
        size_[1] = _n2;
        size_[2] = _n3;
        size_[3] = _n4;
        size_[4] = _n5;
        ensureCapacity(numel());
    }

    void set_size(SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5, SZ _n6) {
        coder::detail::match_dimensions<N == 6>::check();
        size_[0] = _n1;
        size_[1] = _n2;
        size_[2] = _n3;
        size_[3] = _n4;
        size_[4] = _n5;
        size_[5] = _n6;
        ensureCapacity(numel());
    }

    void set_size(SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5, SZ _n6, SZ _n7) {
        coder::detail::match_dimensions<N == 7>::check();
        size_[0] = _n1;
        size_[1] = _n2;
        size_[2] = _n3;
        size_[3] = _n4;
        size_[4] = _n5;
        size_[5] = _n6;
        size_[6] = _n7;
        ensureCapacity(numel());
    }

    void set_size(SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5, SZ _n6, SZ _n7, SZ _n8) {
        coder::detail::match_dimensions<N == 8>::check();
        size_[0] = _n1;
        size_[1] = _n2;
        size_[2] = _n3;
        size_[3] = _n4;
        size_[4] = _n5;
        size_[5] = _n6;
        size_[6] = _n7;
        size_[7] = _n8;
        ensureCapacity(numel());
    }

    void set_size(SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5, SZ _n6, SZ _n7, SZ _n8, SZ _n9) {
        coder::detail::match_dimensions<N == 9>::check();
        size_[0] = _n1;
        size_[1] = _n2;
        size_[2] = _n3;
        size_[3] = _n4;
        size_[4] = _n5;
        size_[5] = _n6;
        size_[6] = _n7;
        size_[7] = _n8;
        size_[8] = _n9;
        ensureCapacity(numel());
    }

    void set_size(SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5, SZ _n6, SZ _n7, SZ _n8, SZ _n9, SZ _n10) {
        coder::detail::match_dimensions<N == 10>::check();
        size_[0] = _n1;
        size_[1] = _n2;
        size_[2] = _n3;
        size_[3] = _n4;
        size_[4] = _n5;
        size_[5] = _n6;
        size_[6] = _n7;
        size_[7] = _n8;
        size_[8] = _n9;
        size_[9] = _n10;
        ensureCapacity(numel());
    }

    template <SizeType N1>
    array_base<T, SZ, N1> reshape_n(const SZ (&_ns)[N1]) const {
        array_base<T, SZ, N1> reshaped(const_cast<T*>(&data_[0]), _ns);
        return reshaped;
    }

    array_base<T, SZ, 1> reshape(SZ _n1) const {
        const SZ ns[] = {_n1};
        return reshape_n(ns);
    }

    array_base<T, SZ, 2> reshape(SZ _n1, SZ _n2) const {
        const SZ ns[] = {_n1, _n2};
        return reshape_n(ns);
    }

    array_base<T, SZ, 3> reshape(SZ _n1, SZ _n2, SZ _n3) const {
        const SZ ns[] = {_n1, _n2, _n3};
        return reshape_n(ns);
    }

    array_base<T, SZ, 4> reshape(SZ _n1, SZ _n2, SZ _n3, SZ _n4) const {
        const SZ ns[] = {_n1, _n2, _n3, _n4};
        return reshape_n(ns);
    }

    array_base<T, SZ, 5> reshape(SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5) const {
        const SZ ns[] = {_n1, _n2, _n3, _n4, _n5};
        return reshape_n(ns);
    }

    array_base<T, SZ, 6> reshape(SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5, SZ _n6) const {
        const SZ ns[] = {_n1, _n2, _n3, _n4, _n5, _n6};
        return reshape_n(ns);
    }

    array_base<T, SZ, 7> reshape(SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5, SZ _n6, SZ _n7) const {
        const SZ ns[] = {_n1, _n2, _n3, _n4, _n5, _n6, _n7};
        return reshape_n(ns);
    }

    array_base<T, SZ, 8> reshape(SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5, SZ _n6, SZ _n7, SZ _n8)
        const {
        const SZ ns[] = {_n1, _n2, _n3, _n4, _n5, _n6, _n7, _n8};
        return reshape_n(ns);
    }

    array_base<T, SZ, 9>
    reshape(SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5, SZ _n6, SZ _n7, SZ _n8, SZ _n9) const {
        const SZ ns[] = {_n1, _n2, _n3, _n4, _n5, _n6, _n7, _n8, _n9};
        return reshape_n(ns);
    }

    array_base<T, SZ, 10>
    reshape(SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5, SZ _n6, SZ _n7, SZ _n8, SZ _n9, SZ _n10) const {
        const SZ ns[] = {_n1, _n2, _n3, _n4, _n5, _n6, _n7, _n8, _n9, _n10};
        return reshape_n(ns);
    }

    T& operator[](SZ _index) {
        return data_[_index];
    }

    const T& operator[](SZ _index) const {
        return data_[_index];
    }

    void clear() {
        data_.clear();
    }

    T* data() {
        return data_;
    }

    const T* data() const {
        return data_;
    }

    const SZ* size() const {
        return &size_[0];
    }

    SZ size(SZ _index) const {
        return size_[_index];
    }

    SZ numel() const {
        return coder::detail::numel<N>::compute(size_);
    }

    SZ index(SZ _n1) const {
        coder::detail::match_dimensions<N == 1>::check();
        const SZ indices[] = {_n1};
        return coder::detail::index_nd<1>::compute(size_, indices);
    }
    SZ index(SZ _n1, SZ _n2) const {
        coder::detail::match_dimensions<N == 2>::check();
        const SZ indices[] = {_n1, _n2};
        return coder::detail::index_nd<2>::compute(size_, indices);
    }
    SZ index(SZ _n1, SZ _n2, SZ _n3) const {
        coder::detail::match_dimensions<N == 3>::check();
        const SZ indices[] = {_n1, _n2, _n3};
        return coder::detail::index_nd<3>::compute(size_, indices);
    }
    SZ index(SZ _n1, SZ _n2, SZ _n3, SZ _n4) const {
        coder::detail::match_dimensions<N == 4>::check();
        const SZ indices[] = {_n1, _n2, _n3, _n4};
        return coder::detail::index_nd<4>::compute(size_, indices);
    }
    SZ index(SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5) const {
        coder::detail::match_dimensions<N == 5>::check();
        const SZ indices[] = {_n1, _n2, _n3, _n4, _n5};
        return coder::detail::index_nd<5>::compute(size_, indices);
    }
    SZ index(SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5, SZ _n6) const {
        coder::detail::match_dimensions<N == 6>::check();
        const SZ indices[] = {_n1, _n2, _n3, _n4, _n5, _n6};
        return coder::detail::index_nd<6>::compute(size_, indices);
    }
    SZ index(SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5, SZ _n6, SZ _n7) const {
        coder::detail::match_dimensions<N == 7>::check();
        const SZ indices[] = {_n1, _n2, _n3, _n4, _n5, _n6, _n7};
        return coder::detail::index_nd<7>::compute(size_, indices);
    }
    SZ index(SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5, SZ _n6, SZ _n7, SZ _n8) const {
        coder::detail::match_dimensions<N == 8>::check();
        const SZ indices[] = {_n1, _n2, _n3, _n4, _n5, _n6, _n7, _n8};
        return coder::detail::index_nd<8>::compute(size_, indices);
    }
    SZ index(SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5, SZ _n6, SZ _n7, SZ _n8, SZ _n9) const {
        coder::detail::match_dimensions<N == 9>::check();
        const SZ indices[] = {_n1, _n2, _n3, _n4, _n5, _n6, _n7, _n8, _n9};
        return coder::detail::index_nd<9>::compute(size_, indices);
    }
    SZ index(SZ _n1, SZ _n2, SZ _n3, SZ _n4, SZ _n5, SZ _n6, SZ _n7, SZ _n8, SZ _n9, SZ _n10)
        const {
        coder::detail::match_dimensions<N == 10>::check();
        const SZ indices[] = {_n1, _n2, _n3, _n4, _n5, _n6, _n7, _n8, _n9, _n10};
        return coder::detail::index_nd<10>::compute(size_, indices);
    }

    T& at(SZ _i1) {
        coder::detail::match_dimensions<N == 1>::check();
        return data_[_i1];
    }
    T& at(SZ _i1, SZ _i2) {
        coder::detail::match_dimensions<N == 2>::check();
        return data_[index(_i1, _i2)];
    }
    T& at(SZ _i1, SZ _i2, SZ _i3) {
        coder::detail::match_dimensions<N == 3>::check();
        return data_[index(_i1, _i2, _i3)];
    }
    T& at(SZ _i1, SZ _i2, SZ _i3, SZ _i4) {
        coder::detail::match_dimensions<N == 4>::check();
        return data_[index(_i1, _i2, _i3, _i4)];
    }
    T& at(SZ _i1, SZ _i2, SZ _i3, SZ _i4, SZ _i5) {
        coder::detail::match_dimensions<N == 5>::check();
        return data_[index(_i1, _i2, _i3, _i4, _i5)];
    }
    T& at(SZ _i1, SZ _i2, SZ _i3, SZ _i4, SZ _i5, SZ _i6) {
        coder::detail::match_dimensions<N == 6>::check();
        return data_[index(_i1, _i2, _i3, _i4, _i5, _i6)];
    }
    T& at(SZ _i1, SZ _i2, SZ _i3, SZ _i4, SZ _i5, SZ _i6, SZ _i7) {
        coder::detail::match_dimensions<N == 7>::check();
        return data_[index(_i1, _i2, _i3, _i4, _i5, _i6, _i7)];
    }
    T& at(SZ _i1, SZ _i2, SZ _i3, SZ _i4, SZ _i5, SZ _i6, SZ _i7, SZ _i8) {
        coder::detail::match_dimensions<N == 8>::check();
        return data_[index(_i1, _i2, _i3, _i4, _i5, _i6, _i7, _i8)];
    }
    T& at(SZ _i1, SZ _i2, SZ _i3, SZ _i4, SZ _i5, SZ _i6, SZ _i7, SZ _i8, SZ _i9) {
        coder::detail::match_dimensions<N == 9>::check();
        return data_[index(_i1, _i2, _i3, _i4, _i5, _i6, _i7, _i8, _i9)];
    }
    T& at(SZ _i1, SZ _i2, SZ _i3, SZ _i4, SZ _i5, SZ _i6, SZ _i7, SZ _i8, SZ _i9, SZ _i10) {
        coder::detail::match_dimensions<N == 10>::check();
        return data_[index(_i1, _i2, _i3, _i4, _i5, _i6, _i7, _i8, _i9, _i10)];
    }

    const T& at(SZ _i1) const {
        coder::detail::match_dimensions<N == 1>::check();
        return data_[_i1];
    }
    const T& at(SZ _i1, SZ _i2) const {
        coder::detail::match_dimensions<N == 2>::check();
        return data_[index(_i1, _i2)];
    }
    const T& at(SZ _i1, SZ _i2, SZ _i3) const {
        coder::detail::match_dimensions<N == 3>::check();
        return data_[index(_i1, _i2, _i3)];
    }
    const T& at(SZ _i1, SZ _i2, SZ _i3, SZ _i4) const {
        coder::detail::match_dimensions<N == 4>::check();
        return data_[index(_i1, _i2, _i3, _i4)];
    }
    const T& at(SZ _i1, SZ _i2, SZ _i3, SZ _i4, SZ _i5) const {
        coder::detail::match_dimensions<N == 5>::check();
        return data_[index(_i1, _i2, _i3, _i4, _i5)];
    }
    const T& at(SZ _i1, SZ _i2, SZ _i3, SZ _i4, SZ _i5, SZ _i6) const {
        coder::detail::match_dimensions<N == 6>::check();
        return data_[index(_i1, _i2, _i3, _i4, _i5, _i6)];
    }
    const T& at(SZ _i1, SZ _i2, SZ _i3, SZ _i4, SZ _i5, SZ _i6, SZ _i7) const {
        coder::detail::match_dimensions<N == 7>::check();
        return data_[index(_i1, _i2, _i3, _i4, _i5, _i6, _i7)];
    }
    const T& at(SZ _i1, SZ _i2, SZ _i3, SZ _i4, SZ _i5, SZ _i6, SZ _i7, SZ _i8) const {
        coder::detail::match_dimensions<N == 8>::check();
        return data_[index(_i1, _i2, _i3, _i4, _i5, _i6, _i7, _i8)];
    }
    const T& at(SZ _i1, SZ _i2, SZ _i3, SZ _i4, SZ _i5, SZ _i6, SZ _i7, SZ _i8, SZ _i9) const {
        coder::detail::match_dimensions<N == 9>::check();
        return data_[index(_i1, _i2, _i3, _i4, _i5, _i6, _i7, _i8, _i9)];
    }
    const T& at(SZ _i1, SZ _i2, SZ _i3, SZ _i4, SZ _i5, SZ _i6, SZ _i7, SZ _i8, SZ _i9, SZ _i10)
        const {
        coder::detail::match_dimensions<N == 10>::check();
        return data_[index(_i1, _i2, _i3, _i4, _i5, _i6, _i7, _i8, _i9, _i10)];
    }

    array_iterator<array_base<T, SZ, N> > begin() {
        return array_iterator<array_base<T, SZ, N> >(this, 0);
    }
    array_iterator<array_base<T, SZ, N> > end() {
        return array_iterator<array_base<T, SZ, N> >(this, this->numel());
    }
    const_array_iterator<array_base<T, SZ, N> > begin() const {
        return const_array_iterator<array_base<T, SZ, N> >(this, 0);
    }
    const_array_iterator<array_base<T, SZ, N> > end() const {
        return const_array_iterator<array_base<T, SZ, N> >(this, this->numel());
    }

  protected:
    coder::detail::data_ptr<T, SZ> data_;
    SZ size_[N];

  private:
    void ensureCapacity(SZ _newNumel) {
        if (_newNumel > data_.capacity()) {
            SZ i = data_.capacity();
            if (i < 16) {
                i = 16;
            }

            while (i < _newNumel) {
                if (i > 1073741823) {
                    i = MAX_int32_T;
                } else {
                    i <<= 1;
                }
            }
            data_.reserve(i);
        }
        data_.resize(_newNumel);
    }
};

// The standard coder::array class with base type and number of dimensions.
template <typename T, int N>
class array : public array_base<T, SizeType, N> {
  private:
    typedef array_base<T, SizeType, N> Base;

  public:
    array()
        : Base() {
    }
    array(const array<T, N>& _other)
        : Base(_other) {
    }
    array(const Base& _other)
        : Base(_other) {
    }
    array(T* _data, const SizeType* _sz)
        : Base(_data, _sz) {
    }
};

// Specialize on char_T (row vector) for better support on strings.
template <>
class array<char_T, 2> : public array_base<char_T, SizeType, 2> {
  private:
    typedef array_base<char_T, SizeType, 2> Base;

  public:
    array()
        : array_base() {
    }
    array(const array<char_T, 2>& _other)
        : Base(_other) {
    }
    array(const Base& _other)
        : Base(_other) {
    }

    array(const std::string& _str) {
        operator=(_str);
    }

    array(const char_T* _str) {
        operator=(_str);
    }

    array(const std::vector<char_T>& _vec) {
        SizeType n = static_cast<SizeType>(_vec.size());
        set_size(1, n);
        data_.copy(&_vec[0], n);
    }

    array& operator=(const std::string& _str) {
        SizeType n = static_cast<SizeType>(_str.size());
        set_size(1, n);
        data_.copy(_str.c_str(), n);
        return *this;
    }

    array& operator=(const char_T* _str) {
        SizeType n = static_cast<SizeType>(strlen(_str));
        set_size(1, n);
        data_.copy(_str, n);
        return *this;
    }

    operator std::string() const {
        return std::string(static_cast<const char*>(&(*this)[0]), static_cast<int>(size(1)));
    }
};

// Specialize on 2 dimensions for better support interactions with
// std::vector and row vectors.
template <typename T>
class array<T, 2> : public array_base<T, SizeType, 2> {
  private:
    typedef array_base<T, SizeType, 2> Base;

  public:
    array()
        : Base() {
    }
    array(const array<T, 2>& _other)
        : Base(_other) {
    }
    array(const Base& _other)
        : Base(_other) {
    }
    array(const std::vector<T>& _vec) {
        operator=(_vec);
    }

    array& operator=(const std::vector<T>& _vec) {
        SizeType n = static_cast<SizeType>(_vec.size());
        Base::set_size(1, n);
        Base::data_.copy(&_vec[0], n);
        return *this;
    }

    operator std::vector<T>() const {
        const T* p = &Base::data_[0];
        return std::vector<T>(p, p + Base::numel());
    }
};

// Specialize on 1 dimension for better support with std::vector and
// column vectors.
template <typename T>
class array<T, 1> : public array_base<T, SizeType, 1> {
  private:
    typedef array_base<T, SizeType, 1> Base;

  public:
    array()
        : Base() {
    }
    array(const array<T, 1>& _other)
        : Base(_other) {
    }
    array(const Base& _other)
        : Base(_other) {
    }
    array(const std::vector<T>& _vec) {
        operator=(_vec);
    }

    array& operator=(const std::vector<T>& _vec) {
        SizeType n = static_cast<SizeType>(_vec.size());
        Base::set_size(n);
        Base::data_.copy(&_vec[0], n);
        return *this;
    }

    operator std::vector<T>() const {
        const T* p = &Base::data_[0];
        return std::vector<T>(p, p + Base::numel());
    }
};
} // namespace coder

#endif
