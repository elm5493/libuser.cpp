
#ifndef USER__ENDIANNESS_HPP
#define USER__ENDIANNESS_HPP

#include <cstring>
#include <cstdint>
#include <type_traits>
#include <stdexcept>
#include <utility>

#ifdef __has_include
#	if __has_include(<bit>)
#		include <bit>
#	endif
#endif

namespace user {
namespace endian {

#ifdef __cpp_lib_endian

	using std::endian;

	constexpr const std::endian native_endianness = std::endian::native;

	inline constexpr std::endian endianness() noexcept {
	 return std::endian::native;
	}

#else
	enum class endian : int {
		unknown,
		little,
		big,
		pdp,
#	if !defined(USER__BIG_ENDIAN) || defined(USER__RASPBERRY_PI)
		native = little,
#	else
		native = bit,
#	endif
	};

	constexpr const endian native_endianness = endian::native;

	namespace {
		inline endian endianness_impl_() noexcept {
		 const std::uint32_t dword = 0x01000200;
		 std::uint8_t byte[4];

		  std::memcpy(byte, &dword, sizeof(dword));

			switch( byte[0] ){
				default  : return endian::unknown;
				case 0x00: return endian::big;
				case 0x01: return endian::little;
				case 0x02: return endian::pdp;
			}

		}
	} // ! namespace [private]


	inline endian endianness() noexcept {
	 static const endian res = endianness_impl_();
	 return res;
	}

#endif


inline constexpr std::uint8_t byte_swap(std::uint8_t x) noexcept {
 return x;
}

inline constexpr std::uint16_t byte_swap(std::uint16_t x) noexcept {
 return (x << 8) | (x >> 8);
}

inline constexpr std::uint32_t byte_swap(std::uint32_t x) noexcept {
  x = ( (x >> 8) & 0x00ff00ff ) | ( (x << 8) & 0xff00ff00 );
 return (x << 16) | (x >> 16);
}

inline constexpr std::uint64_t byte_swap(std::uint64_t x) noexcept {
 return byte_swap(static_cast<std::uint32_t>(x >> 32)) | (static_cast<std::uint64_t>(byte_swap(static_cast<std::uint32_t>(x))) << 32);
}

template <class T>
inline constexpr T byte_swap(const T& x) noexcept {
 return static_cast<T>( byte_swap(static_cast<typename std::make_unsigned<T>::type>(x)) );
}

template <class T>
inline constexpr void byte_swap(T* ptr, std::size_t n) noexcept {
	while( n-- ){
	  *ptr = byte_swap(*ptr);
	  ++ptr;
	}
}

inline constexpr std::uint8_t pdp_swap(std::uint8_t x) noexcept {
 return x;
}

inline constexpr std::uint16_t pdp_swap(std::uint16_t x) noexcept {
 return byte_swap(x);
}

inline constexpr std::uint32_t pdp_swap(std::uint32_t x) noexcept {
 return ( (x >> 8) & 0x00ff00ff ) | ( (x << 8) & 0xff00ff00 );
}

inline constexpr std::uint64_t pdp_swap(std::uint64_t x) noexcept {
 return ( (x >> 8) & static_cast<std::uint64_t>(0x00ff00ff00ff00ff) ) | ( (x << 8) & static_cast<std::uint64_t>(0xff00ff00ff00ff00) );
}

template <class T>
inline constexpr T pdp_swap(const T& x) noexcept {
 return static_cast<T>( pdp_swap(static_cast<typename std::make_unsigned<T>::type>(x)) );
}

template <class T>
inline constexpr void pdp_swap(T* ptr, std::size_t n) noexcept {
	while( n-- ){
	  *ptr = pdp_swap(*ptr);
	  ++ptr;
	}
}


namespace {

	template <endian From, endian To> struct byte_swap_as_needed_impl_ {
		template <class T>
		static constexpr T bswap(const T& x) noexcept {
		  throw std::logic_error("cannot byte swap. this architecture is neither big nor little endian.");
		}
	};

	template <endian E> struct byte_swap_as_needed_impl_<E, E> {
		template <class T>
		static constexpr T bswap(const T& x) noexcept {
		 return x;
		}
		static constexpr void bswap(...) noexcept {}
	};

	template <> struct byte_swap_as_needed_impl_<endian::little, endian::big> {
		template <class ...Args>
		static constexpr auto bswap(Args&&... args) noexcept {
		 return byte_swap( std::forward<Args>(args)... );
		}
	};

	template <> struct byte_swap_as_needed_impl_<endian::big, endian::little>
	 : public byte_swap_as_needed_impl_<endian::little, endian::big>
	{};


} // ! namespace [private]


template <endian E1, endian E2> struct is_same_endian : public std::false_type {};
template <endian E>             struct is_same_endian<E, E> : public std::true_type {};

template <endian E>
struct is_need_byte_swap
 : public std::integral_constant<bool, !is_same_endian<E, native_endianness>::value >
{};

template <endian To, class ...Args>
inline constexpr auto byte_swap_as_needed(Args&&... args) noexcept {
 return byte_swap_as_needed_impl_<native_endianness, To>::bswap( std::forward<Args>(args)... );
}

} // ! namespace endian
} // ! namespace user

#endif // ! USER__ENDIANNESS_HPP

