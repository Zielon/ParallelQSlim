// Copyright 2018 David B. Adrian
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

#ifndef FASTPLY_H
#define FASTPLY_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <tuple>
#include <utility>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace fastply {

/**
 * @brief Returns size (bytes) of a file.
 *
 * @param filename Path to file
 * @return Size in bytes, or -1 upon failure
 */
    inline std::size_t getFileSize(const std::string& filename) noexcept {
        struct ::stat st;
        int rc = stat(filename.c_str(), &st);
        return rc == 0 ? st.st_size : 0;
    }

    template <typename T>
    class PlyElementContainer {
    public:
        using value_type = T;
        using difference_type = std::ptrdiff_t;
        using pointer = T*;
        using const_pointer = const T*;
        using reference = T&;
        using const_reference = const T&;
        using iterator = T*;
        using const_iterator = const T*;

        constexpr const_reference operator[](std::size_t i) const {
            return begin_[i];
        }

        constexpr const_reference at(std::size_t i) const noexcept(false) {
            if (i < size_) {
                return begin_[i];
            } else {
                throw std::out_of_range("Accessed position is out of range");
            }
        }

        constexpr const_reference front() const noexcept { return *begin_; }

        constexpr const_reference back() const noexcept { return *(end_ - 1); }

        constexpr const_pointer data() const noexcept { return begin_; }

        constexpr const_iterator begin() const noexcept { return begin_; }

        constexpr const_iterator cbegin() const noexcept { return begin_; }

        constexpr const_iterator end() const noexcept { return end_; }

        constexpr const_iterator cend() const noexcept { return end_; }

        constexpr std::size_t size() const noexcept { return size_; }

        constexpr bool empty() const noexcept { return size_ == 0; }

    private:
        std::size_t size_ = 0;
        const_pointer begin_ = nullptr;
        const_pointer end_ = nullptr;

        template <typename... Args>
        friend class FastPly;
    };

    template <typename... Args>
    class FastPly {
        static_assert(
                sizeof...(Args),
                "FastPly expects at least one element definition as template parameter.");

    public:
        FastPly() = default;

        ~FastPly() { close(); };

        FastPly(const FastPly&) = delete;
        FastPly& operator=(const FastPly&) = delete;

        bool open(const std::string& path);

        void close();

        std::string getInputPath() const noexcept { return path_; }

        int getHeaderOffset() const noexcept { return header_length_; }

        bool isBigEndian() const noexcept { return is_big_endian_; }

        bool isHeaderParsed() const noexcept { return header_parsed_; }

        std::size_t numberElements() const noexcept { return num_element_definitions; }

        const auto& getElements() const noexcept { return elements_; }

        template <typename T>
        const auto& get() const noexcept {
            return std::get<PlyElementContainer<T>>(elements_);
        }

        template <std::size_t I>
        const auto& get() const noexcept {
            return std::get<I>(elements_);
        }

    private:
        bool parseHeader();

        bool readEncoding(std::istream& is);

        bool readElementDefinition(std::istream& is);

#if defined(__cplusplus) && (__cplusplus == 201402L)
        template <std::size_t idx>
  void setupInnerElementImpl();

  template <std::size_t idx>
  void resetInnerElementImpl();

  template <std::size_t... idx>
  void setupInnerElementDispatcher(std::index_sequence<idx...>);

  template <std::size_t... idx>
  void resetInnerElementDispatcher(std::index_sequence<idx...>);

#endif

        template <typename T, typename... Ts>
        void setupElements();

        template <typename T, typename... Ts>
        void resetElements();

        std::string path_ = "";                //!< Path to input ply file
        bool is_big_endian_ = false;           //!< Encoding of ply file
        std::size_t num_parsed_elements_ = 0;  //!< Elements parsed from ply header
        int header_length_ = -1;               //!< Length of header in bytes
        bool header_parsed_ = false;           //!< Indicates valid header

        std::tuple<PlyElementContainer<Args>...>
                elements_;  //!< Element layouts in order as given as template params
        const unsigned int num_element_definitions =
                sizeof...(Args);  //!< Number of template params (known at compile time)
        std::size_t element_count_[sizeof...(Args)] =
                {};  //!< Num. elements per element definition

        std::size_t file_length_;
        void* ptr_mapped_file_ = nullptr;  //!< Ptr to start of mmap'ed file
    };

    template <typename... Args>
    bool FastPly<Args...>::open(const std::string& path) {
        if (!num_element_definitions)
            return false;

        if (ptr_mapped_file_ != nullptr)  // already opened
            return true;

        path_ = path;

        // Parse Header: This will only query the basic information
        // such as little/big endian encoding, how many elements etc.
        if (!parseHeader())
            return false;

        // Open file descriptor required by mmap
        int fd = ::open(path_.c_str(), O_RDONLY, 0);
        if (fd == -1)
            throw std::system_error(EFAULT, std::generic_category());

        // Get filesize (required by mmap)
        file_length_ = getFileSize(path_.c_str());
        if (file_length_ <= 0)
            throw std::system_error(EFAULT, std::generic_category());

        // memory map the file descriptor
        ptr_mapped_file_ = mmap(0, file_length_, PROT_READ, MAP_PRIVATE, fd, 0);
        ::close(fd); // can be closed
        if (ptr_mapped_file_ == MAP_FAILED) {
            throw std::runtime_error("Failed to memory map " + path_);
        }

        // Fill PlyElementContainers with information (num_elements, ptr offsets etc.)
        setupElements<Args...>();

        return true;
    }

    template <typename... Args>
    void FastPly<Args...>::close() {
        // Freeind mmaped memory
        if (ptr_mapped_file_ != nullptr) {
            if (munmap(ptr_mapped_file_, file_length_) == -1) {
                throw std::runtime_error("Failed to unmap memory!");
            }
            ptr_mapped_file_ = nullptr;
        }

        file_length_ = 0;
        path_ = "";
        is_big_endian_ = false;
        num_parsed_elements_ = 0;
        header_length_ = -1;
        header_parsed_ = false;

        std::fill(element_count_, element_count_ + num_element_definitions, 0);

        resetElements<Args...>();
    }

    template <typename... Args>
    bool FastPly<Args...>::parseHeader() {
        std::ifstream is(path_, std::ios::binary);
        if (is.fail())
            throw std::system_error(EFAULT, std::generic_category());

        std::string line;
        while (std::getline(is, line)) {
            std::istringstream ls(line);
            std::string keyword;
            ls >> keyword;

            // This transformation to all lower chars only handles ASCII.
            // Ply files should only be ASCII (+binary), so it should be alright.
            // Note: This allows slightly non-standard formats to be successfully
            // parsed. aka PlY plY cOmMent, will be fine.
            std::transform(keyword.begin(), keyword.end(), keyword.begin(), ::tolower);

            if (keyword == "ply" || keyword == "")
                continue;
            else if (keyword == "comment")
                continue;
            else if (keyword == "format") {
                if (!readEncoding(ls))
                    return false;
            } else if (keyword == "element") {
                if (!this->readElementDefinition(ls))
                    return false;
            } else if (keyword == "property")
                continue;
            else if (keyword == "obj_info")
                continue;
            else if (keyword == "end_header")
                break;
            else {
                throw std::runtime_error("Unknown keyword '" + keyword + "' found");
                return false;  // exception or bool unexpected header field
            }
        }

        header_length_ = is.tellg();
        if (header_length_ <= 0) {
            if (!is.eof())
                throw std::runtime_error(
                        "Could not determine length of header. Empty file?");
        }

        is.close();
        header_parsed_ = true;
        return true;
    }

    template <typename... Args>
    bool FastPly<Args...>::readEncoding(std::istream& is) {
        std::string s;
        (is >> s);
        if (s == "binary_little_endian")
            is_big_endian_ = false;
        else if (s == "binary_big_endian")
            is_big_endian_ = true;
        else
            return false;  // No support for ascii or typos ;).
        return true;
    }

    template <typename... Args>
    bool FastPly<Args...>::readElementDefinition(std::istream& is) {
        // If we read more element definition keywords than defined
        if (num_parsed_elements_ >= num_element_definitions) {
            throw std::runtime_error(
                    "Definition of PLY file does not match the loaded file. More "
                    "element definitions found than number of template parameters!");
        }

        std::string s;
        (is >> s);  // s contains element name, not being used atm

        // Store number of instances of this element type
        (is >> s);  // s contains element count as string
        element_count_[num_parsed_elements_] = std::stoi(s);  // convert to number

        ++num_parsed_elements_;  // update the count of read elements
        return true;
    }

#if defined(__cplusplus) && (__cplusplus == 201402L)
    template <typename... Args>
template <std::size_t idx>
void FastPly<Args...>::setupInnerElementImpl() {
  auto& el = std::get<idx + 1>(elements_);
  el.size_ = element_count_[idx + 1];
  unsigned char const* start =
      reinterpret_cast<unsigned char const*>(std::get<(idx)>(elements_).end_);
  el.begin_ = reinterpret_cast<decltype(el.begin_)>(start);
  el.end_ = el.begin_ + el.size_;
}

template <typename... Args>
template <std::size_t idx>
void FastPly<Args...>::resetInnerElementImpl() {
  auto& el = std::get<idx>(elements_);
  el.size_ = 0;
  el.begin_ = nullptr;
  el.end_ = nullptr;
}

template <typename... Args>
template <std::size_t... idx>
void FastPly<Args...>::setupInnerElementDispatcher(
    std::index_sequence<idx...>) {
  (setupInnerElementImpl<std::integral_constant<std::size_t, idx>{}>(), ...);
}

template <typename... Args>
template <std::size_t... idx>
void FastPly<Args...>::resetInnerElementDispatcher(
    std::index_sequence<idx...>) {
  (resetInnerElementImpl<std::integral_constant<std::size_t, idx>{}>(), ...);
}

template <typename... Args>
template <typename T, typename... Ts>
void FastPly<Args...>::setupElements() {
  // Setup first element (special case)
  auto& el = std::get<0>(elements_);
  el.size_ = element_count_[0];
  unsigned char const* start =
      static_cast<unsigned char const*>(ptr_mapped_file_) + header_length_;
  el.begin_ = reinterpret_cast<decltype(el.begin_)>(start);
  el.end_ = el.begin_ + el.size_;

  // Setup remaining elements
  auto remaining_indices =
      std::make_index_sequence<sizeof...(Args) - 1>{};
  setupInnerElementDispatcher(remaining_indices);
}

template <typename... Args>
template <typename T, typename... Ts>
void FastPly<Args...>::resetElements() {
  auto indices = std::make_index_sequence<sizeof...(Args)>{};
  resetInnerElementDispatcher(indices);
}

#else

    template <typename... Args>
    template <typename T, typename... Ts>
    void FastPly<Args...>::setupElements() {
        // Index of current element to setup
        constexpr int idx = sizeof...(Args) - sizeof...(Ts) - 1;

        // Get element and fill in the size
        auto& el = std::get<idx>(elements_);
        el.size_ = element_count_[idx];

        unsigned char const* start;
        if constexpr (idx == 0) {
            start =
                    static_cast<unsigned char const*>(ptr_mapped_file_) + header_length_;
        } else {
            start = reinterpret_cast<unsigned char const*>(
                    std::get<(idx - 1)>(elements_).end_);
        }

        el.begin_ = reinterpret_cast<decltype(el.begin_)>(start);
        el.end_ = el.begin_ + el.size_;

        if constexpr (sizeof...(Ts) > 0) {
            setupElements<Ts...>();
        }
    }

    template <typename... Args>
    template <typename T, typename... Ts>
    void FastPly<Args...>::resetElements() {
        // Index of current element to setup
        constexpr int idx = sizeof...(Args) - sizeof...(Ts) - 1;

        // Get element and fill in the size
        auto& el = std::get<idx>(elements_);
        el.size_ = 0;
        el.begin_ = nullptr;
        el.end_ = nullptr;

        if constexpr (sizeof...(Ts) > 0) {
            resetElements<Ts...>();
        }
    }
#endif

}  // namespace fastply

#endif /* FASTPLY_H */