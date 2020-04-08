
#ifndef USER__SPI_HPP
#define USER__SPI_HPP

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include <cstdint>
#include <cstddef>
#include <stdexcept>
#include <system_error>
#include <cerrno>
#include <string>

namespace user {
namespace spi {

class spi_error : public std::system_error {

  public:
	using error_code = std::int32_t;
	static constexpr const error_code bad_error              = 0x80000000;
	static constexpr const error_code fail_error             = 0x40000000;
	static constexpr const error_code succeeded              = 0x00000000;
	static constexpr const error_code unknown_error          = 0xffffffff;
	static constexpr const error_code ioctl_error_speed      = 0x00000001 | bad_error;
	static constexpr const error_code ioctl_error_mode       = 0x00000002 | bad_error;
	static constexpr const error_code ioctl_error_bit_length = 0x00000004 | bad_error;
	static constexpr const error_code ioctl_error_bit_order  = 0x00000008 | bad_error;
	static constexpr const error_code read_failed            = 0x00000010 | bad_error;
	static constexpr const error_code write_failed           = 0x00000020 | bad_error;
	static constexpr const error_code ioctl_error_config     = ioctl_error_speed | ioctl_error_mode | ioctl_error_bit_length | ioctl_error_bit_order | bad_error;
	static constexpr const error_code dispatch_failed        = 0x00000040 | bad_error;
	static constexpr const error_code open_failed            = 0x00000100 | fail_error;
	static constexpr const error_code close_failed           = 0x00000200 | fail_error;
	static constexpr const error_code device_not_opened      = 0x00000400 | fail_error;
	static constexpr const error_code arguments_invalid      = 0x00000800 | fail_error;

	class category : public std::error_category {
	  public:
		static const std::error_category& get();
		const char* name() const noexcept override;
		std::string message(spi_error::error_code ec) const override;
	};

	static error_code check(int res);
	static bool assert(int res, error_code state = succeeded);

  private:
	std::error_condition ec_;

  public:
	spi_error();
	explicit spi_error(const std::string& what);
	explicit spi_error(int err_no, error_code code, const std::string& what = "");

	virtual const std::error_condition& condition() const noexcept;

};


enum class spi_mode : int {
	mode_unknown = -1,
	mode_0 = SPI_MODE_0,
	mode_1 = SPI_MODE_1,
	mode_2 = SPI_MODE_2,
	mode_3 = SPI_MODE_3,
};

enum class bit_order : std::uint8_t {
	lsb,
	msb,
};

enum class byte_swap {
	swap,
	noswap,
	as_needed,
};


class spi {

  public:
	struct param {
		spi_mode  mode;
		std::uint32_t  speed;
		std::uint8_t   bit_length;
		bit_order order;

		constexpr param();
		constexpr param(spi_mode mode, std::uint32_t speed, std::uint8_t bit_length, bit_order order);
	};


	class msg {
	  private:
		struct spi_ioc_transfer tr_;

	  public:

		constexpr msg();

		void set_speed(std::uint32_t hz) noexcept;
		std::uint32_t get_speed() const noexcept;

		void set_bit_length(std::uint8_t bit_length) noexcept;
		std::uint8_t get_bit_length() const noexcept;

		struct spi_ioc_transfer& set_buffer(void* rx, const void* tx, std::size_t length) noexcept;
		struct spi_ioc_transfer& set_rx_buffer(void* rx) noexcept;
		struct spi_ioc_transfer& set_tx_buffer(const void* tx) noexcept;
		struct spi_ioc_transfer& set_length(std::size_t length) noexcept;
		const struct spi_ioc_transfer& get() const noexcept;

	};


  private:
	using self = spi;

	int fd_;
	msg message_;
	spi_error::error_code status_;

	spi_error::error_code update_status_(spi_error::error_code res) noexcept;

  public:

	static self open_stream(const char* device, const param& param);
	static self open_stream(
		const char* device,
		spi_mode mode,
		std::uint32_t speed = 1000000,
		std::uint8_t bit_length = 8,
		bit_order order = bit_order::lsb
	);

	spi();
	explicit spi(const char* device, const param& param);
	explicit spi(
		const char* device,
		spi_mode mode,
		std::uint32_t speed = 1000000,
		std::uint8_t bit_length = 8,
		bit_order order = bit_order::lsb
	);
	~spi();

	spi(const self& cp) = delete;
	spi(self&& mv) noexcept;

	int open(const char* device) noexcept;
	int close() noexcept;

	operator bool() const noexcept;
	bool operator !() const noexcept;
	bool fail() const noexcept;
	bool good() const noexcept;
	bool bad() const noexcept;
	void clear() noexcept;
	bool is_open() const noexcept;

	int get_param(spi::param& p) noexcept;
	int set_param(const spi::param& p) noexcept;

	int get_mode(spi_mode& mode) noexcept;
	int set_mode(spi_mode mode) noexcept;

	int set_speed(std::uint32_t hz) noexcept;
	int get_speed(std::uint32_t& hz) noexcept;

	int set_bit_length(std::uint8_t bit_length) noexcept;
	int get_bit_length(std::uint8_t& bit_length) noexcept;

	int set_bit_order(bit_order order) noexcept;
	int get_bit_order(bit_order& order) noexcept;

	int dispatch(const struct spi_ioc_transfer& message) noexcept;

	template <std::size_t N>
	int dispatch(const struct spi_ioc_transfer(&arr)[N]) noexcept;

	template <class T>
	int dispatch(T* rx, const T* tx, std::size_t n) noexcept;
	int dispatch(const msg& message) noexcept;

	int write(const void* buf, std::size_t length) noexcept;

	template <class T, std::size_t N>
	int write(const T(&buf)[N]) noexcept;

	int read(void* buf, std::size_t length) noexcept;

	template <class T, std::size_t N>
	int read(T(&buf)[N]) noexcept;

	self& operator =(const self& cp) = delete;
	self& operator =(self&& mv) noexcept;

};




} // ! namespace spi
} // ! namespace user

#include "impl/spi_impl.hpp"

#endif // ! USER__SPI_HPP

