

#ifndef USER__I2C_IMPL_HPP
#define USER__I2C_IMPL_HPP

namespace user {
namespace i2c {

/* function smbus api */

inline std::int32_t i2c_smbus_access(int file, char read_write, std::uint8_t command, int size, union i2c_smbus_data* data) {
 struct i2c_smbus_ioctl_data args;
  args.read_write = read_write;
  args.command = command;
  args.size = size;
  args.data = data;
 return ::ioctl(file, I2C_SMBUS, &args);
}


inline std::uint32_t i2c_smbus_write_quick(int file, std::uint8_t value) {
 return i2c_smbus_access(file, value, 0, I2C_SMBUS_QUICK, NULL);
}

inline std::uint32_t i2c_smbus_read_byte(int file) {
 union i2c_smbus_data data;
	if (i2c_smbus_access(file, I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE,&data)){
	 return -1;
	}else{
	 return 0x0FF & data.byte;
	}
}

inline std::uint32_t i2c_smbus_write_byte(int file, std::uint8_t value) {
 return i2c_smbus_access(file, I2C_SMBUS_WRITE, value, I2C_SMBUS_BYTE, NULL);
}

inline std::uint32_t i2c_smbus_read_byte_data(int file, std::uint8_t command) {
 union i2c_smbus_data data;
	if (i2c_smbus_access(file, I2C_SMBUS_READ, command, I2C_SMBUS_BYTE_DATA,&data)){
	 return -1;
	}else{
	 return 0x0FF & data.byte;
	}
}

inline std::uint32_t i2c_smbus_write_byte_data(int file, std::uint8_t command, std::uint8_t value) {
 union i2c_smbus_data data;
  data.byte = value;
 return i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_BYTE_DATA, &data);
}

inline std::uint32_t i2c_smbus_read_word_data(int file, std::uint8_t command) {
 union i2c_smbus_data data;
	if (i2c_smbus_access(file, I2C_SMBUS_READ, command, I2C_SMBUS_WORD_DATA,&data)){
	 return -1;
	}else{
	 return 0x0FFFF & data.word;
	}
}

inline std::uint32_t i2c_smbus_write_word_data(int file, std::uint8_t command, std::uint16_t value) {
 union i2c_smbus_data data;
  data.word = value;
 return i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_WORD_DATA, &data);
}

inline std::uint32_t i2c_smbus_process_call(int file, std::uint8_t command, std::uint16_t value) {
 union i2c_smbus_data data;

  data.word = value;

	if (i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_PROC_CALL,&data)){
	 return -1;
	}else{
	 return 0x0FFFF & data.word;
	}
}


/* Returns the number of read bytes */
inline std::uint32_t i2c_smbus_read_block_data(int file, std::uint8_t command, std::uint8_t* values) {
 union i2c_smbus_data data;
 int i;
	if (i2c_smbus_access(file, I2C_SMBUS_READ, command, I2C_SMBUS_BLOCK_DATA,&data)){
	 return -1;
	}else{
		for (i = 1; i <= data.block[0]; i++){
		  values[i-1] = data.block[i];
		}
	 return data.block[0];
	}
}

inline std::uint32_t i2c_smbus_write_block_data(int file, std::uint8_t command, std::uint8_t length, std::uint8_t* values) {
 union i2c_smbus_data data;
 int i;

	if (length > I2C_SMBUS_BLOCK_MAX) length = I2C_SMBUS_BLOCK_MAX;

	for (i = 1; i <= length; i++){
	  data.block[i] = values[i-1];
	}

  data.block[0] = length;

 return i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_BLOCK_DATA, &data);
}


inline std::uint32_t i2c_smbus_read_i2c_block_data(int file, std::uint8_t command, std::uint8_t length, std::uint8_t* values) {
 union i2c_smbus_data data;
 int i, err;

	if( length > I2C_SMBUS_BLOCK_MAX ) length = I2C_SMBUS_BLOCK_MAX;

  data.block[0] = length;
  err = i2c_smbus_access(file, I2C_SMBUS_READ, command, (length == 32) ? I2C_SMBUS_I2C_BLOCK_BROKEN : I2C_SMBUS_I2C_BLOCK_DATA, &data);

	if( err < 0 ) return err;

	for(i=1 ; i <= data.block[0] ; ++i){
	  values[i - 1] = data.block[i];
	}

 return data.block[0];
}


inline std::uint32_t i2c_smbus_write_i2c_block_data(int file, std::uint8_t command, std::uint8_t length, std::uint8_t* values) {
 union i2c_smbus_data data;
 int i;

	if (length > I2C_SMBUS_BLOCK_MAX) length = I2C_SMBUS_BLOCK_MAX;

	for (i = 1; i <= length; i++){
	  data.block[i] = values[i-1];
	}

  data.block[0] = length;

 return i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_I2C_BLOCK_BROKEN, &data);
}


inline std::uint32_t i2c_smbus_block_process_call(int file, std::uint8_t command, std::uint8_t length, std::uint8_t* values) {
 union i2c_smbus_data data;
 int i, err;

	if( length > I2C_SMBUS_BLOCK_MAX ) length = I2C_SMBUS_BLOCK_MAX;

	for(i=1 ; i <= length ; i++){
	  data.block[i] = values[i-1];
	}

  data.block[0] = length;
  err = i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_BLOCK_PROC_CALL, &data);

	if( err < 0 ) return err;

	for(i=1 ; i <= data.block[0] ; ++i){
	  values[i-1] = data.block[i];
	}

 return data.block[0];
}


/* class i2c_error */

inline i2c_error::error_code i2c_error::check(int res) {
	if( res >= 0 ) return res;
	switch( res ){
		default                        : throw i2c_error(errno, unknown_error);
		case open_failed               : throw i2c_error(errno, open_failed);
		case device_not_opened         : throw i2c_error(errno, device_not_opened);
		case addressing_failed         : throw i2c_error(errno, addressing_failed);
		case closing_failed            : throw i2c_error(errno, closing_failed);
		case dispatch_failed           : throw i2c_error(errno, dispatch_failed);
		case read_failed               : throw i2c_error(errno, read_failed);
		case write_failed              : throw i2c_error(errno, write_failed);
		case enable_pec_failed         : throw i2c_error(errno, enable_pec_failed);
		case disable_pec_failed        : throw i2c_error(errno, disable_pec_failed);
		case smbus_process_call_failed : throw i2c_error(errno, smbus_process_call_failed);
	}
}

inline bool i2c_error::assert(int res, int state) {
	if( res != state ) throw i2c_error("Assertion error");
	else               return res;
}


inline i2c_error::i2c_error() : i2c_error(0, 0, "") {}
inline i2c_error::i2c_error(const char* what) : i2c_error(0, 0, what) {}
inline i2c_error::i2c_error(int error_no, error_code errcode, const char* what)
 : system_error( std::error_code(error_no, std::generic_category()), what )
 , ec_( errcode, i2c_error::category::get() )
{}

inline const std::error_condition& i2c_error::condition() const noexcept {
 return ec_;
}


inline const std::error_category& i2c_error::category::get() noexcept {
 static const i2c_error::category cat{};
 return cat;
}


/* class i2c_error::category */

inline const char* i2c_error::category::name() const noexcept { return "i2c"; }

inline std::string i2c_error::category::message(int ev) const {
	switch( ev ){
		default                                   : return "Unknown error";
		case i2c_error::open_failed               : return "Open failed";
		case i2c_error::device_not_opened         : return "Device not opened";
		case i2c_error::addressing_failed         : return "Addressing failed";
		case i2c_error::closing_failed            : return "Close failed";
		case i2c_error::dispatch_failed           : return "Dispatch failed";
		case i2c_error::read_failed               : return "Read failed";
		case i2c_error::write_failed              : return "Write failed";
		case i2c_error::enable_pec_failed         : return "Enable PEC failed";
		case i2c_error::disable_pec_failed        : return "Disable PEC failed";
		case i2c_error::smbus_process_call_failed : return "SMBus process call failed";
		case i2c_error::read_features_failed      : return "Read features failed";
	}
}


/* class i2c::reg */

template <std::size_t N>
inline constexpr i2c::reg<N>::reg(reg_type val, i2c::byte_swap s)
 : val_(
	(s == i2c::byte_swap::noswap) ? val
	: (s == i2c::byte_swap::swap) ? endian::byte_swap<reg_type>(val)
	: endian::byte_swap_as_needed<endian::endian::big>( static_cast<reg_type>(val) )
 )
{}

template <std::size_t N>
inline constexpr i2c::reg<N> i2c::reg<N>::bswap() const noexcept {
 return self( endian::byte_swap<reg_type>(val_) );
}

template <std::size_t N>
inline constexpr typename i2c::reg<N>::reg_type i2c::reg<N>::value() const noexcept {
 return val_;
}

template <std::size_t N>
inline const std::uint8_t* i2c::reg<N>::data() const noexcept { return reinterpret_cast<const std::uint8_t*>(&val_); }


/* class i2c::msg */

inline constexpr i2c::msg::msg() : msg(0x0000, 0, 0, nullptr) {}

inline constexpr i2c::msg::msg(std::uint16_t dev_addr, std::uint16_t flags)
 : msg(dev_addr, flags, 0, nullptr)
{}

inline constexpr i2c::msg::msg(
 std::uint16_t dev_addr,
 std::uint16_t flags,
 std::uint16_t length,
 std::uint8_t* buf
)
 : message_{ dev_addr, flags, length, buf }
{}

inline struct i2c_msg i2c::msg::write_message(
 const void* buf,
 std::uint16_t len
) const noexcept {
 struct i2c_msg message;
  message.addr = message_.addr;
  message.flags = message_.flags & 0xfffe;
  message.len = len;
  message.buf = const_cast<std::uint8_t*>(reinterpret_cast<const std::uint8_t*>(buf));
 return message;
}

inline struct i2c_msg i2c::msg::read_message(
 void* buf,
 std::uint16_t len
) const noexcept {
 struct i2c_msg message;
  message.addr = message_.addr;
  message.flags = message_.flags | I2C_M_RD;
  message.len = len;
  message.buf = reinterpret_cast<std::uint8_t*>(buf);
 return message;
}

inline const struct i2c_msg& i2c::msg::get() const noexcept {
 return message_;
}

inline struct i2c_msg& i2c::msg::set_addr(std::uint16_t addr) noexcept {
  message_.addr = addr;
 return message_;
}

inline struct i2c_msg& i2c::msg::set_flags(std::uint16_t flags) noexcept {
  message_.flags |= flags;
 return message_;
}

inline struct i2c_msg& i2c::msg::unset_flags(std::uint16_t flags) noexcept {
  message_.flags &= (~flags);
 return message_;
}

inline struct i2c_msg& i2c::msg::set_buf(std::uint16_t len, std::uint8_t* buf) noexcept {
  message_.len = len;
  message_.buf = buf;
 return message_;
}

template <class T, std::size_t N>
inline struct i2c_msg& i2c::msg::set_buf(T(&arr)[N]) noexcept {
  message_.len = N * sizeof(T);
  message_.buf = reinterpret_cast<std::uint8_t*>(arr);
 return message_;
}

inline std::uint16_t i2c::msg::get_addr() const noexcept { return message_.addr; }
inline std::uint16_t i2c::msg::get_flags() noexcept { return message_.flags; }
inline std::uint16_t i2c::msg::get_len() const noexcept { return message_.len; }
inline std::uint8_t* i2c::msg::get_buf() noexcept { return message_.buf; }


/* class i2c */

std::mutex i2c::mut_;
std::unique_lock<std::mutex> i2c::lock_{i2c::mut_, std::defer_lock};

template <class T>
inline void i2c::byte_swap_copy_(
 std::uint8_t* p,
 const T* data,
 std::size_t n
) noexcept {
 T tmp;
	while( n-- ){
	  tmp = endian::byte_swap(*data);
	  std::memcpy(p, &tmp, sizeof(T));
	  ++data;
	  p += sizeof(T);
	}
}

template <class T>
inline void i2c::byte_swap_copy_as_needed_(
 std::uint8_t* p,
 const T* data,
 std::size_t n
) noexcept {
 T tmp;
	while( n-- ){
	  tmp = endian::byte_swap_as_needed<endian::endian::big>(*data);
	  std::memcpy(p, &tmp, sizeof(T));
	  ++data;
	  p += sizeof(T);
	}
}


inline i2c_error::error_code i2c::update_status_(i2c_error::error_code code) noexcept {
  status_ |= code;
 return code;
}


inline bool i2c::try_lock() {
	if( !lock_.owns_lock() ){
	 return lock_.try_lock();
	}
}

inline void i2c::unlock() {
	if( lock_.owns_lock() ) lock_.unlock();
}

/*
template <std::size_t N>
inline constexpr auto i2c::reg(typename byte_<N>::type addr, byte_swap swap) {
	switch( swap ){
		case byte_swap::swap  : return reg<N>( endian::byte_swap_as_needed<endian::endian::big>(addr) );
		case byte_swap::noswap: return reg<N>( addr );
	}
}
*/

inline i2c i2c::open_stream(const char* device, std::uint16_t address) {
 i2c inst;
  i2c_error::check( inst.open(device) );
  i2c_error::check( inst.set_address(address) );
 return inst;
}

inline i2c::i2c()
 : fd_(-1)
 , message_()
 , status_(i2c_error::device_not_opened)
 , ssobuf_{}
{}

inline i2c::i2c(const char* device)
 : i2c()
{
  self::open(device);
}

inline i2c::i2c(const char* device, std::uint16_t address)
 : i2c()
{
  self::open(device);
  set_address(address);
}

inline i2c::i2c(self&& mv) noexcept
 : fd_(mv.fd_)
 , message_(mv.message_)
{
  mv.fd_ = -1;
  mv.message_ = msg();
  mv.status_ = i2c_error::device_not_opened;
}

inline i2c::~i2c() { 
  self::close();
}

inline int i2c::open(const char* device) {
	if( !is_open() ){
	 int res = self::close();
		if( res != i2c_error::succeeded ) return update_status_(res);
	}

  fd_ = ::open(device, O_RDWR);

	if( fd_ < 0 ) return update_status_(i2c_error::open_failed);
	else          clear();

 return i2c_error::succeeded;
}

inline int i2c::close() noexcept {
	if( !is_open() )        return update_status_(i2c_error::device_not_opened);
	if( 0 != ::close(fd_) ) return update_status_(i2c_error::closing_failed);
  fd_ = -1;
  clear();
 return i2c_error::succeeded;
}


inline bool i2c::operator !() const noexcept {
 return fail();
}

inline i2c::operator bool() const noexcept {
 return !fail();
}

inline bool i2c::fail() const noexcept {
 return status_ != i2c_error::succeeded;
}

inline void i2c::clear() noexcept {
	if( is_open() ) status_ = i2c_error::succeeded;
	else            status_ = i2c_error::device_not_opened;
}

inline int i2c::set_address(std::uint16_t address) noexcept {
	if( !is_open() ) return update_status_(i2c_error::device_not_opened);
	else             message_.set_addr(address);
	if( -1 == ::ioctl(fd_, I2C_SLAVE, address) ) return update_status_(i2c_error::addressing_failed);
	else                                        return i2c_error::succeeded;
}

inline int i2c::select_address_mode(bit_mode mode) noexcept {
	if( !is_open() ) return update_status_(i2c_error::device_not_opened);
	switch( mode ){
		case bit_mode::bit7:  message_.unset_flags(static_cast<std::uint16_t>(mode)); break;
		case bit_mode::bit10: message_.set_flags(static_cast<std::uint16_t>(mode)); break;
	}
	if( -1 == ::ioctl(fd_, I2C_TENBIT, static_cast<long>(mode)) ) return update_status_(i2c_error::addressing_failed);
	else                                                          return i2c_error::succeeded;
}

inline int i2c::enable_smbus_pec() noexcept {
	if( !is_open() ) return update_status_(i2c_error::device_not_opened);
	if( -1 == ::ioctl(fd_, I2C_PEC, 1) ) return update_status_(i2c_error::enable_pec_failed);
	else                                 return i2c_error::succeeded;
}

inline int i2c::disable_smbus_pec() noexcept {
	if( !is_open() ) return update_status_(i2c_error::device_not_opened);
	if( -1 == ::ioctl(fd_, I2C_PEC, 0) ) return update_status_(i2c_error::disable_pec_failed);
	else                                 return i2c_error::succeeded;
}

inline int i2c::read_features(unsigned long& res) noexcept {
	if( !is_open() ) return update_status_(i2c_error::device_not_opened);
	if( -1 == ::ioctl(fd_, I2C_FUNCS, &res) ) return update_status_(i2c_error::read_features_failed);
	else                                      return i2c_error::succeeded;
}

inline bool i2c::is_feature_enabled(std::uint32_t feature, i2c_error::error_code* res) noexcept {
 const std::uint16_t feature_value = static_cast<std::uint16_t>(feature);
 unsigned long features;
 i2c_error::error_code result = read_features(features);
	if( res != nullptr ) *res = result;
	if( features & feature_value == feature_value ) return true;
	else                                            return false;
}

inline std::uint16_t i2c::get_address() const noexcept {
 return message_.get_addr();
}

inline bool i2c::is_open() const noexcept {
 return fd_ >= 0;
}

inline int i2c::dispatch(const struct i2c_msg* msgs, std::size_t n) noexcept {
 struct i2c_rdwr_ioctl_data data = { const_cast<struct i2c_msg*>(msgs), n };
	if( !is_open() ) return update_status_(i2c_error::device_not_opened);
	if( 2 != ::ioctl(fd_, I2C_RDWR, &data) ) return update_status_(i2c_error::dispatch_failed);
	else                                     return i2c_error::succeeded;
}

template <std::size_t N>
inline int i2c::dispatch(const struct i2c_msg(&msgs)[N]) noexcept {
 return dispatch(msgs, N);
}

inline int i2c::dispatch(const struct i2c_msg& msg) noexcept {
 return dispatch(&msg, 1);
}


template <std::size_t RegSize, class T>
inline int i2c::read(
 const reg<RegSize>& reg_addr,
 T* rx, std::size_t n,
 byte_swap swap
) noexcept {
 struct i2c_msg messages[] = {
	message_.write_message(reg_addr.data(), RegSize),
	message_.read_message(rx, n * sizeof(T))
 };
 int res = dispatch(messages);

	if( res == i2c_error::succeeded ){
		switch( swap ){
			case byte_swap::swap      : endian::byte_swap(rx, n); break;
			case byte_swap::as_needed : endian::byte_swap_as_needed<endian::endian::big>(rx, n); break;
		}
	 return i2c_error::succeeded;
	}else if( res == i2c_error::dispatch_failed ){
	 return update_status_(i2c_error::read_failed);
	}else{
	 return update_status_(res);
	}

}

template <class T>
inline int i2c::read(T* rx, std::size_t n, byte_swap swap) noexcept {
 int res = dispatch( message_.read_message(rx, n * sizeof(T)) );

	if( res == i2c_error::succeeded ){
		switch( swap ){
			case byte_swap::swap      : endian::byte_swap(rx, n); break;
			case byte_swap::as_needed : endian::byte_swap_as_needed<endian::endian::big>(rx, n); break;
		}
	 return i2c_error::succeeded;
	}else if( res == i2c_error::dispatch_failed ){
	 return update_status_(i2c_error::read_failed);
	}else{
	 return update_status_(res);
	}

}

template <std::size_t RegSize, class T>
inline int i2c::read(const reg<RegSize>& reg_addr, T& rx, byte_swap swap) noexcept {
 return read(reg_addr, &rx, 1, swap);
}

template <class T>
inline int i2c::read(T& rx, byte_swap swap) noexcept {
 return read(&rx, 1, swap);
}

template <std::size_t RegSize, class T, std::size_t N>
inline int i2c::read(const reg<RegSize>& reg_addr, T(&arr)[N], byte_swap swap) noexcept {
 return read(reg_addr, arr, N, swap);
}

template <class T, std::size_t N>
inline int i2c::read(T(&arr)[N], byte_swap swap) noexcept {
 return read(arr, N, swap);
}


template <std::size_t RegSize, class T>
inline int i2c::write(const reg<RegSize>& reg_addr, const T* tx, std::size_t n, byte_swap swap) noexcept {
 const std::size_t buf_size = n * sizeof(T) + RegSize;
 int res;

	if( buf_size > ssobuf_size ){
	 std::uint8_t* ptr = new std::uint8_t[ buf_size ];
	  std::memcpy(ptr, reg_addr.data(), RegSize);

		switch( swap ){
			case byte_swap::noswap    : std::memcpy(ptr + RegSize, tx, n * sizeof(T)); break;
			case byte_swap::swap      : byte_swap_copy_(ptr + RegSize, tx, n); break;
			case byte_swap::as_needed : byte_swap_copy_as_needed_(ptr + RegSize, tx, n); break;
		}

	  res = dispatch( message_.write_message(ptr, buf_size) );
	  delete[] ptr;
	}else{
	  std::memcpy(ssobuf_, reg_addr.data(), RegSize);

		switch( swap ){
			case byte_swap::noswap    : std::memcpy(ssobuf_ + RegSize, tx, n * sizeof(T)); break;
			case byte_swap::swap      : byte_swap_copy_(ssobuf_ + RegSize, tx, n); break;
			case byte_swap::as_needed : byte_swap_copy_as_needed_(ssobuf_ + RegSize, tx, n); break;
		}

	  res = dispatch( message_.write_message(ssobuf_, buf_size) );
	}

	if( res == i2c_error::dispatch_failed ) return update_status_(i2c_error::write_failed);
	else                                    return update_status_(res);

}


template <class T>
inline int i2c::write(const T* tx, std::size_t n, byte_swap swap) noexcept {
 int res;

	if( swap == byte_swap::noswap ){
	  res = dispatch( message_.write_message(tx, n * sizeof(T)) );
	}else{
	 const std::size_t buf_size = n * sizeof(T);

		if( buf_size > ssobuf_size ){
		 std::uint8_t* ptr = new std::uint8_t[ buf_size ];
			switch( swap ){
				case byte_swap::swap      : byte_swap_copy_(ptr, tx, n); break;
				case byte_swap::as_needed : byte_swap_copy_as_needed_(ptr, tx, n); break;
			}
		  res = dispatch( message_.write_message(ptr, buf_size) );
		  delete[] ptr;
		}else{
			switch( swap ){
				case byte_swap::swap      : byte_swap_copy_(ssobuf_, tx, n); break;
				case byte_swap::as_needed : byte_swap_copy_as_needed_(ssobuf_, tx, n); break;
			}
		  res = dispatch( message_.write_message(ssobuf_, buf_size) );
		}
	}

	if( res == i2c_error::dispatch_failed ) return update_status_(i2c_error::write_failed);
	else                                    return update_status_(res);
}

template <std::size_t RegSize, class T>
inline int i2c::write(const reg<RegSize>& reg_addr, const T& tx, byte_swap swap) noexcept {
 constexpr const std::size_t buf_size = RegSize + sizeof(T);
 std::uint8_t buf[ buf_size ];
 int res;

  std::memcpy(buf, reg_addr.data(), RegSize);

	switch( swap ){
		case byte_swap::noswap    : std::memcpy(buf, &tx, sizeof(T)); break;
		case byte_swap::swap      : byte_swap_copy_(buf, &tx, 1); break;
		case byte_swap::as_needed : byte_swap_copy_as_needed_(buf, &tx, 1); break;
	}

  res = dispatch( message_.write_message(buf, buf_size) );

	if( res == i2c_error::dispatch_failed ) return update_status_(i2c_error::write_failed);
	else                                    return update_status_(res);
}


template <class T>
inline int i2c::write(const T& tx, byte_swap swap) noexcept {
	switch( swap ){
		case byte_swap::noswap    : return write(&tx, 1, byte_swap::noswap); break;
		case byte_swap::swap      : return write(&endian::byte_swap(tx), 1, byte_swap::noswap); break;
		case byte_swap::as_needed : return write(&endian::byte_swap_as_needed<endian::endian::big>(tx), 1, byte_swap::noswap); break;
	}
}


template <std::size_t RegSize, class T, std::size_t N>
inline int i2c::write(const reg<RegSize>& reg_addr, const T(&arr)[N], byte_swap swap) noexcept {
 constexpr const std::size_t buf_size = N * sizeof(T) + RegSize;
 std::uint8_t buf[ buf_size ];
 int res;

  std::memcpy(buf, reg_addr.data(), RegSize);

	switch( swap ){
		case byte_swap::noswap    : std::memcpy(buf + RegSize, arr, N * sizeof(T)); break;
		case byte_swap::swap      : byte_swap_copy_(buf + RegSize, arr, N); break;
		case byte_swap::as_needed : byte_swap_copy_as_needed_(buf + RegSize, arr, N); break;
	}

  res = dispatch( message_.write_message(buf, buf_size) );

	if( res == i2c_error::dispatch_failed ) return update_status_(i2c_error::write_failed);
	else                                    return update_status_(res);

}

template <class T, std::size_t N>
inline int i2c::write(const T(&arr)[N], byte_swap swap) noexcept {
 constexpr const std::size_t buf_size = N * sizeof(T);
 int res;

	if( swap == byte_swap::noswap ) {
	  res = dispatch( message_.write_message(arr, buf_size) );
	}else{
	 std::uint8_t buf[ buf_size ];

		switch( swap ){
			case byte_swap::swap      : byte_swap_copy_(buf, arr, N); break;
			case byte_swap::as_needed : byte_swap_copy_as_needed_(buf, arr, N); break;
		}

	  res = dispatch( message_.write_message(buf, buf_size) );
	}

	if( res == i2c_error::dispatch_failed ) return update_status_(i2c_error::write_failed);
	else                                    return update_status_(res);

}


inline i2c& i2c::operator =(i2c&& mv) noexcept {
	if( this != &mv ){
	  close();
	  fd_ = mv.fd_;
	  message_ = mv.message_;
	  status_ = mv.status_;
	  mv.fd_ = -1;
	  mv.message_ = i2c::msg();
	  mv.status_ = i2c_error::device_not_opened;
	}
 return *this;
}



inline int i2c::smbus_write_quick(std::uint8_t value) noexcept {
	if( !is_open() ) return update_status_(i2c_error::device_not_opened);
	if( -1 == i2c_smbus_write_quick(fd_, value) ) return update_status_(i2c_error::write_failed);
	else                                          return i2c_error::succeeded;
}

inline int i2c::smbus_read_byte(std::uint8_t& value) noexcept {
	if( !is_open() ){
	 return update_status_(i2c_error::device_not_opened);
	}else{
	 std::int32_t res = i2c_smbus_read_byte(fd_);
		if( res == -1 ) return update_status_(i2c_error::read_failed);
		else            value = res & 0xff;
	 return i2c_error::succeeded;
	}
}

inline int i2c::smbus_write_byte(std::uint8_t value) noexcept {
	if( !is_open() ) return update_status_(i2c_error::device_not_opened);
	if( -1 == i2c_smbus_write_byte(fd_, value) ) return update_status_(i2c_error::write_failed);
	else                                         return i2c_error::succeeded;
}

inline int i2c::smbus_read_byte_data(std::uint8_t reg, std::uint8_t& value) noexcept {
	if( !is_open() ){
	 return update_status_(i2c_error::device_not_opened);
	}else{
	 std::int32_t res = i2c_smbus_read_byte_data(fd_, reg);
		if( res == -1 ) return update_status_(i2c_error::read_failed);
		else            value = res & 0xff;
	 return i2c_error::succeeded;
	}
}

inline int i2c::smbus_write_byte_data(std::uint8_t reg, std::uint8_t value) noexcept {
	if( !is_open() ) return update_status_(i2c_error::device_not_opened);
	if( -1 == i2c_smbus_write_byte_data(fd_, reg, value) ) return update_status_(i2c_error::write_failed);
	else                                                   return i2c_error::succeeded;
}

inline int i2c::smbus_read_word_data(std::uint8_t reg, std::uint16_t& value) noexcept {
	if( !is_open() ){
	 return update_status_(i2c_error::device_not_opened);
	}else{
	 std::int32_t res = i2c_smbus_read_byte_data(fd_, reg);
		if( res == -1 ) return update_status_(i2c_error::read_failed);
		else            value = res & 0xffff;
	 return i2c_error::succeeded;
	}
}

inline int i2c::smbus_write_byte_data(std::uint8_t reg, std::uint16_t value) noexcept {
	if( !is_open() ) return update_status_(i2c_error::device_not_opened);
	if( -1 == i2c_smbus_write_byte_data(fd_, reg, value) ) return update_status_(i2c_error::write_failed);
	else                                                   return i2c_error::succeeded;
}


inline int i2c::smbus_process_call(std::uint8_t reg, std::uint16_t value) noexcept {
	if( !is_open() ){
	 return update_status_(i2c_error::device_not_opened);
	}else{
	 std::int32_t res = i2c_smbus_process_call(fd_, reg, value);
		if( res == -1 ) return update_status_(i2c_error::smbus_process_call_failed);
		else            value = res & 0xffff;
	 return i2c_error::succeeded;
	}
}


inline int i2c::smbus_read_block_data(std::uint8_t reg, std::uint8_t* values) noexcept {
	if( !is_open() ){
	 return update_status_(i2c_error::device_not_opened);
	}else{
	 std::int32_t res = i2c_smbus_read_block_data(fd_, reg, values);
		if( res == -1 ) return update_status_(i2c_error::read_failed);
	 return res;
	}
}

inline int i2c::smbus_write_block_data(std::uint8_t reg, std::uint8_t length, std::uint8_t* values) noexcept {
	if( !is_open() ) return update_status_(i2c_error::device_not_opened);
	if( -1 == i2c_smbus_write_block_data(fd_, reg, length, values) ){
	 return update_status_(i2c_error::write_failed);
	}else{
	 return i2c_error::succeeded;
	}
}

inline int i2c::smbus_read_i2c_block_data(std::uint8_t reg, std::uint8_t length, std::uint8_t* values) noexcept {
	if( !is_open() ){
	 return update_status_(i2c_error::device_not_opened);
	}else{
	 std::int32_t res = i2c_smbus_read_block_data(fd_, reg, values);
		if( res == -1 ) return update_status_(i2c_error::read_failed);
	 return res;
	}
}

inline int i2c::smbus_write_i2c_block_data(std::uint8_t reg, std::uint8_t length, std::uint8_t* values) noexcept {
	if( !is_open() ) return update_status_(i2c_error::device_not_opened);
	if( -1 == i2c_smbus_write_block_data(fd_, reg, length, values) ){
	 return update_status_(i2c_error::read_failed);
	}else{
	 return i2c_error::succeeded;
	}
}

inline int i2c::smbus_block_process_call(std::uint8_t reg, std::uint8_t length, std::uint8_t* values) noexcept {
	if( !is_open() ){
	 return update_status_(i2c_error::device_not_opened);
	}else{
	 std::int32_t res = i2c_smbus_block_process_call(fd_, reg, length, values);
		if( res == -1 ) return update_status_(i2c_error::smbus_process_call_failed);
		else            return res;
	}
}


} // ! namespace i2c
} // ! namespace user

#endif // ! USER__I2C_IMPL_HPP

