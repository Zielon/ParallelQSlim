// Copyright 2019 David B. Adrian
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once

#include <tuple>

#define FASTPLY_GENERATE_OPERATORS(name, args...)        \
                                                         \
  auto tie_internals_() const { return std::tie(args); } \
                                                         \
  bool operator<(const name& rhs) const {                \
    return tie_internals_() < rhs.tie_internals_();      \
  }                                                      \
                                                         \
  bool operator>(const name& rhs) const {                \
    return tie_internals_() > rhs.tie_internals_();      \
  }                                                      \
                                                         \
  bool operator<=(const name& rhs) const {               \
    return tie_internals_() <= rhs.tie_internals_();     \
  }                                                      \
                                                         \
  bool operator>=(const name& rhs) const {               \
    return tie_internals_() >= rhs.tie_internals_();     \
  }                                                      \
                                                         \
  bool operator==(const name& rhs) const {               \
    return tie_internals_() == rhs.tie_internals_();     \
  }                                                      \
                                                         \
  bool operator!=(const name& rhs) const {               \
    return tie_internals_() != rhs.tie_internals_();     \
  }

#if defined(_MSC_VER)
#define FASTPLY_PACKED_(__FP_ELEMENT__) \
  __pragma(pack(push, 1)) __FP_ELEMENT__ __pragma(pack(pop))
#elif defined(__GNUC__)
#define FASTPLY_PACKED_(__FP_ELEMENT__) \
  __FP_ELEMENT__ __attribute__((__packed__))
#endif

#define FASTPLY_ELEMENT(name, ...) FASTPLY_PACKED_(struct name{__VA_ARGS__});