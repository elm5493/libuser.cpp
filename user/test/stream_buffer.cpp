
#include "user/stream_buffer.hpp"

#include <iostream>
#include <thread>

using namespace user;

using stream_buffer_type = stream_buffer::stream_buffer<int>;


void test_thread(const stream_buffer_type& buffer) {
 auto reader = buffer.get_reader();
 int values[16];

std::cout << "ERRCHK : test_thread::OK" << std::endl;

	while( !reader.is_end() ){
std::cout << "ERRCHK : test_thread::before reader::read" << std::endl;
	  reader.read(values, 16);

std::cout << "ERRCHK : test_thread::after reader::read" << std::endl;

		for( const auto& x : values ){
		  std::cout << "x : " << (unsigned)x << ", ";
		}
	}

  std::cout << "reader ended." << std::endl;
}


int main(void){
 stream_buffer_type buf(16);
std::cout << "ERRCHK : OK" << std::endl;
 std::thread th(test_thread, std::ref(buf));
 int value = 0;
 int count;

	while( true ) {
	  std::cout << "count : ";
	  std::cin >> count;

		if( count == 0 ) break;

		for(int i=0 ; i < count ; ++i){
		  buf.write(&i, 1);
		}
	}

  buf.flip(stream_buffer::policy::sequence);
  buf.end();
  th.join();

  std::cout << "finish." << std::endl;

 return 0;
}



