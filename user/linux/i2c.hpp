
#ifndef USER__I2C_HPP
#define USER__I2C_HPP

#include "user/endianness.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <cerrno>
#include <system_error>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <memory>
#include <mutex>

/* Compatibility defines */
#ifndef I2C_SMBUS_I2C_BLOCK_BROKEN
#define I2C_SMBUS_I2C_BLOCK_BROKEN I2C_SMBUS_I2C_BLOCK_DATA
#endif
#ifndef I2C_FUNC_SMBUS_PEC
#define I2C_FUNC_SMBUS_PEC I2C_FUNC_SMBUS_HWPEC_CALC
#endif


namespace user {
namespace i2c {

namespace {

template <std::size_t> struct byte_;
template <> struct byte_<1> { using type = std::uint8_t; };
template <> struct byte_<2> { using type = std::uint16_t; };
template <> struct byte_<4> { using type = std::uint32_t; };
template <> struct byte_<8> { using type = std::uint64_t; };

} // ! namespace [private]


inline std::int32_t i2c_smbus_access(int file, char read_write, std::uint8_t command, int size, union i2c_smbus_data* data);
inline std::uint32_t i2c_smbus_write_quick(int file, std::uint8_t value);
inline std::uint32_t i2c_smbus_read_byte(int file);
inline std::uint32_t i2c_smbus_write_byte(int file, std::uint8_t value);
inline std::uint32_t i2c_smbus_read_byte_data(int file, std::uint8_t command);
inline std::uint32_t i2c_smbus_write_byte_data(int file, std::uint8_t command, std::uint8_t value);
inline std::uint32_t i2c_smbus_read_word_data(int file, std::uint8_t command);
inline std::uint32_t i2c_smbus_write_word_data(int file, std::uint8_t command, std::uint16_t value);
inline std::uint32_t i2c_smbus_process_call(int file, std::uint8_t command, std::uint16_t value);
inline std::uint32_t i2c_smbus_read_block_data(int file, std::uint8_t command, std::uint8_t* values);
inline std::uint32_t i2c_smbus_write_block_data(int file, std::uint8_t command, std::uint8_t length, std::uint8_t* values);
inline std::uint32_t i2c_smbus_read_i2c_block_data(int file, std::uint8_t command, std::uint8_t length, std::uint8_t* values);
inline std::uint32_t i2c_smbus_write_i2c_block_data(int file, std::uint8_t command, std::uint8_t length, std::uint8_t* values);
inline std::uint32_t i2c_smbus_block_process_call(int file, std::uint8_t command, std::uint8_t length, std::uint8_t* values);


class i2c_error : public std::system_error {

  public:

	using error_code = std::int32_t;
	static constexpr const error_code succeeded                 = 0x00000000;
	static constexpr const error_code unknown_error             = 0xffffffff;
	static constexpr const error_code open_failed               = 0x80000001;
	static constexpr const error_code device_not_opened         = 0x80000002;
	static constexpr const error_code addressing_failed         = 0x80000004;
	static constexpr const error_code closing_failed            = 0x80000008;
	static constexpr const error_code dispatch_failed           = 0x80000010;
	static constexpr const error_code read_failed               = 0x80000020;
	static constexpr const error_code write_failed              = 0x80000040;
	static constexpr const error_code enable_pec_failed         = 0x80000080;
	static constexpr const error_code disable_pec_failed        = 0x80000100;
	static constexpr const error_code smbus_process_call_failed = 0x80000200;
	static constexpr const error_code read_features_failed      = 0x80000400;

	class category : public std::error_category {
	  public:
		static const std::error_category& get() noexcept;
		const char* name() const noexcept;
		std::string message(int ev) const;
	};

	static error_code check(int res);
	static bool assert(int res, int state = succeeded);

  private:
	std::error_condition ec_;

  public:

	i2c_error();
	explicit i2c_error(const char* what);
	explicit i2c_error(int error_no, error_code errcode, const char* what = "");

	virtual const std::error_condition& condition() const noexcept;

};


class i2c {

  public:
	static constexpr const std::uint32_t func_i2c = I2C_FUNC_I2C;
	static constexpr const std::uint32_t func_10bit_addr = I2C_FUNC_10BIT_ADDR;
	static constexpr const std::uint32_t func_protocol_mangling = I2C_FUNC_PROTOCOL_MANGLING;
	static constexpr const std::uint32_t func_smbus_pec = I2C_FUNC_SMBUS_PEC;
	static constexpr const std::uint32_t func_nostart = I2C_FUNC_NOSTART;
	static constexpr const std::uint32_t func_slave = I2C_FUNC_SLAVE;
	static constexpr const std::uint32_t func_smbus_block_proc_call = I2C_FUNC_SMBUS_BLOCK_PROC_CALL;
	static constexpr const std::uint32_t func_smbus_quick = I2C_FUNC_SMBUS_QUICK;
	static constexpr const std::uint32_t func_smbus_read_byte = I2C_FUNC_SMBUS_READ_BYTE;
	static constexpr const std::uint32_t func_smbus_write_byte = I2C_FUNC_SMBUS_WRITE_BYTE;
	static constexpr const std::uint32_t func_smbus_read_byte_data = I2C_FUNC_SMBUS_READ_BYTE_DATA;
	static constexpr const std::uint32_t func_smbus_write_byte_data = I2C_FUNC_SMBUS_WRITE_BYTE_DATA;
	static constexpr const std::uint32_t func_smbus_read_word_data = I2C_FUNC_SMBUS_READ_WORD_DATA;
	static constexpr const std::uint32_t func_smbus_write_word_data = I2C_FUNC_SMBUS_WRITE_WORD_DATA;
	static constexpr const std::uint32_t func_smbus_proc_call = I2C_FUNC_SMBUS_PROC_CALL;
	static constexpr const std::uint32_t func_smbus_read_block_data = I2C_FUNC_SMBUS_READ_BLOCK_DATA;
	static constexpr const std::uint32_t func_smbus_write_block_data = I2C_FUNC_SMBUS_WRITE_BLOCK_DATA;
	static constexpr const std::uint32_t func_smbus_read_i2c_block = I2C_FUNC_SMBUS_READ_I2C_BLOCK;
	static constexpr const std::uint32_t func_smbus_write_i2c_block = I2C_FUNC_SMBUS_WRITE_I2C_BLOCK;
	static constexpr const std::uint32_t func_smbus_host_notify = I2C_FUNC_SMBUS_HOST_NOTIFY;
	static constexpr const std::uint32_t func_smbus_byte = I2C_FUNC_SMBUS_BYTE;
	static constexpr const std::uint32_t func_smbus_byte_data = I2C_FUNC_SMBUS_BYTE_DATA;
	static constexpr const std::uint32_t func_smbus_word_data = I2C_FUNC_SMBUS_WORD_DATA;
	static constexpr const std::uint32_t func_smbus_block_data = I2C_FUNC_SMBUS_BLOCK_DATA;
	static constexpr const std::uint32_t func_smbus_i2c_block = I2C_FUNC_SMBUS_I2C_BLOCK;
	static constexpr const std::uint32_t func_smbus_emul = I2C_FUNC_SMBUS_EMUL;

	static constexpr const std::size_t ssobuf_size = 34;

	enum class bit_mode : std::uint16_t {
		bit7 = 0x0000,
		bit10 = 0x0010,
	};

	enum class byte_swap {
		swap,
		noswap,
		as_needed,
	};

	template <std::size_t N>
	class reg {
	  public:
		using reg_type = typename byte_<N>::type;

	  private:
		using self = reg<N>;

		reg_type val_;

	  public:
		constexpr reg(reg_type val, i2c::byte_swap s = i2c::byte_swap::as_needed);
		constexpr self bswap() const noexcept;
		constexpr reg_type value() const noexcept;
		const std::uint8_t* data() const noexcept;
	};


	class msg {
	  private:
		struct i2c_msg message_;

	  public:
		constexpr msg();
		constexpr msg(std::uint16_t dev_addr, std::uint16_t flags = 0);
		constexpr msg(
		  std::uint16_t dev_addr,
		  std::uint16_t flags,
		  std::uint16_t length,
		  std::uint8_t* buf
		);

		struct i2c_msg write_message(
		  const void* buf,
		  std::uint16_t len
		) const noexcept;

		inline struct i2c_msg read_message(
		  void* buf,
		  std::uint16_t len
		) const noexcept;


		const struct i2c_msg& get() const noexcept;

		struct i2c_msg& set_addr(std::uint16_t addr) noexcept;
		struct i2c_msg& set_flags(std::uint16_t flags) noexcept;
		struct i2c_msg& unset_flags(std::uint16_t flags) noexcept;

		struct i2c_msg& set_buf(std::uint16_t len, std::uint8_t* buf) noexcept;

		template <class T, std::size_t N>
		struct i2c_msg& set_buf(T(&arr)[N]) noexcept;

		std::uint16_t get_addr() const noexcept;
		std::uint16_t get_flags() noexcept;
		std::uint16_t get_len() const noexcept;
		std::uint8_t* get_buf() noexcept;
	};

  private:
	using self = i2c;

	template <class T>
	static void byte_swap_copy_(std::uint8_t* p, const T* data, std::size_t n) noexcept;

	template <class T>
	static void byte_swap_copy_as_needed_(
	 std::uint8_t* p,
	 const T* data,
	 std::size_t n
	) noexcept;

	static std::mutex mut_;
	static std::unique_lock<std::mutex> lock_;

	int fd_;
	msg message_;
	i2c_error::error_code status_;
	mutable std::uint8_t ssobuf_[ssobuf_size];

	i2c_error::error_code update_status_(i2c_error::error_code code) noexcept;

  public:

	static bool try_lock();
	static void unlock();

	//template <std::size_t N>
	//static constexpr auto reg(typename byte_<N>::type addr, byte_swap swap = byte_swap::as_needed);

	static i2c open_stream(const char* device, std::uint16_t address);

	i2c();
	explicit i2c(const char* device);
	explicit i2c(const char* device, std::uint16_t address);
	i2c(const self& cp) = delete;
	i2c(self&& mv) noexcept;
	~i2c();

	int open(const char* device);
	int close() noexcept;

	bool operator !() const noexcept;
	operator bool() const noexcept;
	bool fail() const noexcept;
	void clear() noexcept;

	int set_address(std::uint16_t address) noexcept;
	int select_address_mode(bit_mode mode) noexcept;

	int enable_smbus_pec() noexcept;
	int disable_smbus_pec() noexcept;

	int read_features(unsigned long& res) noexcept;

	bool is_feature_enabled(
	  std::uint32_t feature,
	  i2c_error::error_code* res = nullptr
	) noexcept;

	std::uint16_t get_address() const noexcept;
	bool is_open() const noexcept;

	int dispatch(const struct i2c_msg* msgs, std::size_t n) noexcept;

	template <std::size_t N>
	int dispatch(const struct i2c_msg(&msgs)[N]) noexcept;
	int dispatch(const struct i2c_msg& msg) noexcept;

	template <std::size_t RegSize, class T>
	int read(
	 const reg<RegSize>& reg_addr,
	 T* rx, std::size_t n,
	 byte_swap swap = byte_swap::as_needed
	) noexcept;

	template <class T>
	int read(
	 T* rx,
	 std::size_t n,
	 byte_swap swap = byte_swap::as_needed
	) noexcept;

	template <std::size_t RegSize, class T>
	int read(
	 const reg<RegSize>& reg_addr,
	 T& rx,
	 byte_swap swap = byte_swap::as_needed
	) noexcept;

	template <class T>
	int read(
	 T& rx,
	 byte_swap swap = byte_swap::as_needed
	) noexcept;

	template <std::size_t RegSize, class T, std::size_t N>
	int read(
	 const reg<RegSize>& reg_addr,
	 T(&arr)[N],
	 byte_swap swap = byte_swap::as_needed
	) noexcept;

	template <class T, std::size_t N>
	int read(
	 T(&arr)[N],
	 byte_swap swap = byte_swap::as_needed
	) noexcept;

	template <std::size_t RegSize, class T>
	int write(
	 const reg<RegSize>& reg_addr,
	 const T* tx, std::size_t n,
	 byte_swap swap = byte_swap::as_needed
	) noexcept;

	template <class T>
	int write(
	 const T* tx,
	 std::size_t n,
	 byte_swap swap = byte_swap::as_needed
	) noexcept;

	template <std::size_t RegSize, class T>
	int write(
	 const reg<RegSize>& reg_addr,
	 const T& tx,
	 byte_swap swap = byte_swap::as_needed
	) noexcept;

	template <class T>
	int write(const T& tx, byte_swap swap = byte_swap::as_needed) noexcept;

	template <std::size_t RegSize, class T, std::size_t N>
	int write(
	 const reg<RegSize>& reg_addr,
	 const T(&arr)[N],
	 byte_swap swap = byte_swap::as_needed
	) noexcept;

	template <class T, std::size_t N>
	int write(const T(&arr)[N], byte_swap swap = byte_swap::as_needed) noexcept;

	self& operator =(const self& cp) = delete;
	self& operator =(self&& mv) noexcept;

	int smbus_write_quick(std::uint8_t value) noexcept;
	int smbus_read_byte(std::uint8_t& value) noexcept;
	int smbus_write_byte(std::uint8_t value) noexcept;
	int smbus_read_byte_data(std::uint8_t reg, std::uint8_t& value) noexcept;
	int smbus_write_byte_data(std::uint8_t reg, std::uint8_t value) noexcept;
	int smbus_read_word_data(std::uint8_t reg, std::uint16_t& value) noexcept;
	int smbus_write_byte_data(std::uint8_t reg, std::uint16_t value) noexcept;
	int smbus_process_call(std::uint8_t reg, std::uint16_t value) noexcept;
	int smbus_read_block_data(std::uint8_t reg, std::uint8_t* values) noexcept;
	int smbus_write_block_data(std::uint8_t reg, std::uint8_t length, std::uint8_t* values) noexcept;
	int smbus_read_i2c_block_data(std::uint8_t reg, std::uint8_t length, std::uint8_t* values) noexcept;
	int smbus_write_i2c_block_data(std::uint8_t reg, std::uint8_t length, std::uint8_t* values) noexcept;
	int smbus_block_process_call(std::uint8_t reg, std::uint8_t length, std::uint8_t* values) noexcept;

};

} // ! namespace i2c
} // ! namespace user

#include "./impl/i2c_impl.hpp"

#endif // ! USER__I2C_HPP

