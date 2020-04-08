
#ifndef USER__SPI_IMPL_HPP
#define USER__SPI_IMPL_HPP

namespace user {
namespace spi {

/* class spi_error::category */

inline const std::error_category& spi_error::category::get() {
 static const spi_error::category cat;
 return cat;
}

inline const char* spi_error::category::name() const noexcept {
 return "spi";
}

inline std::string spi_error::category::message(spi_error::error_code ec) const {
	switch( ec ) {
		default                     : return "Unknown error";
		case ioctl_error_speed      : return "IOCTL error about speed r/w";
		case ioctl_error_mode       : return "IOCTL error about mode r/w";
		case ioctl_error_bit_length : return "IOCTL error about bit length r/w";
		case ioctl_error_bit_order  : return "IOCTL error about bit order r/w";
		case ioctl_error_config     : return "IOCTL error about spi config r/w";
		case dispatch_failed        : return "Dispatch failed";
		case read_failed            : return "Read failed";
		case write_failed           : return "Write failed";
		case open_failed            : return "Open failed";
		case close_failed           : return "Close failed";
		case device_not_opened      : return "Device not opened";
		case arguments_invalid      : return "Arguments invalid";
	}
}


/* class spi_error */


inline spi_error::error_code spi_error::check(int res) {
	if( res >= 0 ) return res;
	else           throw spi_error(errno, res);
}


inline bool spi_error::assert(int res, error_code state) {
	if( res == state ) return true;
	else               throw spi_error("Assertion error");
}


inline spi_error::spi_error() : spi_error(0, 0, "") {}

inline spi_error::spi_error(const std::string& what) : spi_error(0, 0, what) {}

inline spi_error::spi_error(int err_no, spi_error::error_code code, const std::string& what)
 : system_error( std::error_code(err_no, std::generic_category()), what )
 , ec_( code, spi_error::category::get() )
{}

inline const std::error_condition& spi_error::condition() const noexcept {
 return ec_;
}



/* class spi::param */

inline constexpr spi::param::param()
 : mode(spi_mode::mode_unknown)
 , speed(1000000)
 , bit_length(8)
 , order(bit_order::lsb)
{}

inline constexpr spi::param::param(spi_mode mode, std::uint32_t speed, std::uint8_t bit_length, bit_order order)
 : mode(mode)
 , speed(speed)
 , bit_length(bit_length)
 , order(order)
{}


/* class spi::msg */

inline constexpr spi::msg::msg() : tr_{} {}

inline void spi::msg::set_speed(std::uint32_t hz) noexcept {
  tr_.speed_hz = hz;
}

inline std::uint32_t spi::msg::get_speed() const noexcept {
 return tr_.speed_hz;
}

inline void spi::msg::set_bit_length(std::uint8_t bit_length) noexcept {
  tr_.bits_per_word = bit_length;
}

inline std::uint8_t spi::msg::get_bit_length() const noexcept {
 return tr_.bits_per_word;
}

inline struct spi_ioc_transfer& spi::msg::set_buffer(void* rx, const void* tx, std::size_t length) noexcept {
  tr_.rx_buf = reinterpret_cast<unsigned long>(rx);
  tr_.tx_buf = reinterpret_cast<unsigned long>(tx);
  tr_.len = length;
 return tr_;
}

inline struct spi_ioc_transfer& spi::msg::set_rx_buffer(void* rx) noexcept {
  tr_.rx_buf = reinterpret_cast<unsigned long>(rx);
 return tr_;
}


inline struct spi_ioc_transfer& spi::msg::set_tx_buffer(const void* tx) noexcept {
  tr_.tx_buf = reinterpret_cast<unsigned long>(tx);
 return tr_;
}


inline struct spi_ioc_transfer& spi::msg::set_length(std::size_t length) noexcept {
  tr_.len = length;
 return tr_;
}


inline const struct spi_ioc_transfer& spi::msg::get() const noexcept {
 return tr_;
}



/* class spi */

inline spi spi::open_stream(const char* device, const param& param) {
 spi res;

  spi_error::check( res.open(device) );
  spi_error::check( res.set_param(param) );

 return res;
}

inline spi spi::open_stream(
	const char* device,
	spi_mode mode,
	std::uint32_t speed,
	std::uint8_t bit_length,
	bit_order order
) {
 spi res;

  spi_error::check( res.open(device) );
  spi_error::check( res.set_mode(mode) );
  spi_error::check( res.set_speed(speed) );
  spi_error::check( res.set_bit_length(bit_length) );
  spi_error::check( res.set_bit_order(order) );

 return res;
}



inline spi::spi()
 : fd_(-1)
 , message_()
 , status_(spi_error::device_not_opened)
{}

inline spi::spi(const char* device, const param& param)
 : spi(device, param.mode, param.speed, param.bit_length, param.order)
{}

inline spi::spi(
 const char* device,
 spi_mode mode,
 std::uint32_t speed,
 std::uint8_t bit_length,
 bit_order order
)
 : spi()
{
  open(device);
  set_mode(mode);
  set_speed(speed);
  set_bit_length(bit_length);
  set_bit_order(order);
}


inline spi::~spi() {
  close();
}


inline spi::spi(self&& mv) noexcept
 : fd_(mv.fd_)
 , message_(mv.message_)
 , status_(mv.status_)
{
  mv.fd_ = -1;
  mv.message_ = spi::msg{};
  mv.status_ = spi_error::device_not_opened;
}


inline spi_error::error_code spi::update_status_(spi_error::error_code res) noexcept {
  status_ |= res;
 return res;
}


inline int spi::open(const char* device) noexcept {

	if( is_open() ) {
	 int res = close();
		if( res != spi_error::succeeded ) return res;
	}

  fd_ = ::open(device, O_RDWR);

	if( fd_ < 0 ) {
	 return update_status_(spi_error::open_failed);
	}else{
	  clear();
	}

 return spi_error::succeeded;
}

inline int spi::close() noexcept {

	if( !is_open() ) return update_status_(spi_error::device_not_opened);

	if( 0 != ::close(fd_) ){
	 return update_status_(spi_error::close_failed);
	}else{
	  fd_ = -1;
	  clear();
	 return spi_error::succeeded;
	}

}


inline spi::operator bool() const noexcept {
 return !fail();
}

inline bool spi::operator !() const noexcept {
 return fail();
}

inline bool spi::fail() const noexcept {
	if( status_ & spi_error::fail_error ) return true;
 return bad();
}

inline bool spi::good() const noexcept {
 return ((status_ & (spi_error::bad_error | spi_error::fail_error)) == 0);
}

inline bool spi::bad() const noexcept {
 return (status_ & spi_error::bad_error);
}

inline void spi::clear() noexcept {
	if( is_open() ) status_ = spi_error::succeeded;
	else            status_ = spi_error::device_not_opened;
}

inline bool spi::is_open() const noexcept {
 return (fd_ >= 0);
}

inline int spi::get_param(spi::param& p) noexcept {
 spi_error::error_code res = 0;

  res |= get_mode(p.mode);
  res |= get_speed(p.speed);
  res |= get_bit_length(p.bit_length);
  res |= get_bit_order(p.order);

	if( res == spi_error::succeeded ) return update_status_(spi_error::ioctl_error_config);
	else                              return spi_error::succeeded;
}


inline int spi::set_param(const spi::param& p) noexcept {
 spi_error::error_code res = 0;

  res |= set_mode(p.mode);
  res |= set_speed(p.speed);
  res |= set_bit_length(p.bit_length);
  res |= set_bit_order(p.order);

	if( res == spi_error::succeeded ) return update_status_(spi_error::ioctl_error_config);
	else                              return spi_error::succeeded;
}


inline int spi::set_mode(spi_mode mode) noexcept {
	if( !is_open() ){
	 return update_status_(spi_error::device_not_opened);
	}else{
	 unsigned int val;
		switch( mode ){
			default: return spi_error::arguments_invalid;
			case spi_mode::mode_0: val = SPI_MODE_0; break;
			case spi_mode::mode_1: val = SPI_MODE_1; break;
			case spi_mode::mode_2: val = SPI_MODE_2; break;
			case spi_mode::mode_3: val = SPI_MODE_3; break;
		}

		if( -1 == ioctl(fd_, SPI_IOC_WR_MODE, &val) ){
		 return update_status_(spi_error::ioctl_error_mode);
		}else{
		 return spi_error::succeeded;
		}
	}
}



inline int spi::get_mode(spi_mode& mode) noexcept {
	if( !is_open() ) {
	 return update_status_(spi_error::device_not_opened);
	}else{
	 std::uint8_t val;

		if( -1 == ioctl(fd_, SPI_IOC_RD_MODE, &val) ) {
		 return update_status_(spi_error::ioctl_error_mode);
		}else{
			switch( val ){
				default         : mode = spi_mode::mode_unknown; break;
				case SPI_MODE_0 : mode = spi_mode::mode_0; break;
				case SPI_MODE_1 : mode = spi_mode::mode_1; break;
				case SPI_MODE_2 : mode = spi_mode::mode_2; break;
				case SPI_MODE_3 : mode = spi_mode::mode_3; break;
			}
		}

	 return spi_error::succeeded;
	}
}



inline int spi::set_speed(std::uint32_t hz) noexcept {
	if( !is_open() ) return update_status_(spi_error::device_not_opened);
	if( -1 == ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &hz) ){
	 return update_status_(spi_error::ioctl_error_speed);
	}else{
	  message_.set_speed(hz);
	 return spi_error::succeeded;
	}
}


inline int spi::get_speed(std::uint32_t& hz) noexcept {
	if( !is_open() ) return update_status_(spi_error::device_not_opened);
	if( -1 == ioctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, &hz) ) {
	 return update_status_(spi_error::ioctl_error_speed);
	}else{
	 return spi_error::succeeded;
	}
}


inline int spi::set_bit_length(std::uint8_t bit_length) noexcept {
	if( !is_open() ) return update_status_(spi_error::device_not_opened);
	if( -1 == ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bit_length) ){
	 return update_status_(spi_error::ioctl_error_bit_length);
	}else{
	  message_.set_bit_length(bit_length);
	 return spi_error::succeeded;
	}
}


inline int spi::get_bit_length(std::uint8_t& bit_length) noexcept {
	if( !is_open() ) return update_status_(spi_error::device_not_opened);
	if( 0 > ioctl(fd_, SPI_IOC_RD_BITS_PER_WORD, &bit_length) ){
	 return update_status_(spi_error::ioctl_error_bit_length);
	}else{
	 return spi_error::succeeded;
	}
}


inline int spi::set_bit_order(bit_order order) noexcept {
	if( !is_open() ){
	 return update_status_(spi_error::device_not_opened);
	}else{
	 std::size_t val;

		switch( order ){
			case bit_order::lsb: val = 0; break;
			case bit_order::msb: val = 1; break;
		}

		if( -1 ==  ioctl(fd_, SPI_IOC_WR_LSB_FIRST, &val) ){
		 return update_status_(spi_error::ioctl_error_bit_order);
		}else{
		 return spi_error::succeeded;
		}
	}
}


inline int spi::get_bit_order(bit_order& order) noexcept {
	if( !is_open() ){
	 return update_status_(spi_error::device_not_opened);
	}else{
	 std::uint8_t res;

		if( 0 > ioctl(fd_, SPI_IOC_RD_LSB_FIRST, &res) ){
		 return update_status_(spi_error::ioctl_error_bit_order);
		}else{
			switch( res ){
				case 0: order = bit_order::lsb; break;
				case 1: order = bit_order::msb; break;
			}
		}

	 return spi_error::succeeded;
	}
}


inline int spi::dispatch(const struct spi_ioc_transfer& message) noexcept {
	if( !is_open() ) return update_status_(spi_error::device_not_opened);
	if( -1 == ::ioctl(fd_, SPI_IOC_MESSAGE(1), &message) ){
	 return update_status_(spi_error::dispatch_failed);
	}else{
	 return spi_error::succeeded;
	}
}


template <std::size_t N>
inline int spi::dispatch(const struct spi_ioc_transfer(&arr)[N]) noexcept {
	if( !is_open() ) return update_status_(spi_error::device_not_opened);
	if( -1 == ::ioctl(fd_, SPI_IOC_MESSAGE(N), arr) ){
	 return update_status_(spi_error::dispatch_failed);
	}else{
	 return spi_error::succeeded;
	}
}


template <class T>
inline int spi::dispatch(T* rx, const T* tx, std::size_t n) noexcept {
 return dispatch( message_.set_buffer(rx, tx, n * sizeof(T)) );
}


inline int spi::dispatch(const spi::msg& message) noexcept {
 return dispatch(message.get());
}


inline int spi::write(const void* buf, std::size_t length) noexcept {
	if( !is_open() ) return update_status_(spi_error::device_not_opened);
	if( length != ::write(fd_, buf, length) ){
	 return update_status_(spi_error::write_failed);
	}else{
	 return spi_error::succeeded;
	}
}


template <class T, std::size_t N>
inline int spi::write(const T(&arr)[N]) noexcept {
 constexpr const std::size_t buffer_size = N * sizeof(T);
 return write(arr, buffer_size);
}



inline int spi::read(void* buf, std::size_t length) noexcept {
	if( !is_open() ) return update_status_(spi_error::device_not_opened);
	if( -1 == ::read(fd_, buf, length) ){
	 return update_status_(spi_error::read_failed);
	}else{
	 return spi_error::succeeded;
	}
}



template <class T, std::size_t N>
inline int spi::read(T(&buf)[N]) noexcept {
 constexpr const std::size_t buffer_size = N * sizeof(T);
 return read(buf, buffer_size);
}


inline spi& spi::operator =(spi&& mv) noexcept {
	if( &mv != this ){
	  close();
	  fd_ = mv.fd_;
	  message_ = mv.message_;
	  status_ = mv.status_;
	  mv.fd_ = -1;
	  mv.message_ = spi::msg();
	  mv.status_ = spi_error::device_not_opened;
	}
 return *this;
}

} // ! namespace spi
} // ! namespace user

#endif // ! USER__SPI_IMPL_HPP

