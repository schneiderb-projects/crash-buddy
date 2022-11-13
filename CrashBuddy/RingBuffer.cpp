#include <Arduino.h>
#include "RingBuffer.h"

RingBuffer::RingBuffer(void* buffer, uint32_t sizeof_buffer, uint8_t sizeof_data) {
  _buffer = (char*) buffer;
  _sizeof_data = sizeof_data;
  _index = 0;
  _sizeof_buffer = sizeof_buffer / sizeof_data * sizeof_data; //allign the buffer size to a multiple of the data size 
}

void RingBuffer::push(void* data) {
  memcpy((_buffer + _index), data, _sizeof_data);
  _index = (_index + _sizeof_data) % _sizeof_buffer;
}

void RingBuffer::pop(uint32_t offset, uint32_t size, void* to_fill) { // pop "size" many elements starting at ("_index" + "offset") and place them in "to_fill"
	uint32_t filled = 0; 
	uint32_t index = (_index + offset * _sizeof_data) % _sizeof_buffer;
	uint32_t bytes_remaining = size * _sizeof_data;
	while (bytes_remaining) {
		if ( _sizeof_buffer - index >= bytes_remaining) {
			memcpy((char *) to_fill + filled, _buffer + index, bytes_remaining);
			bytes_remaining = 0;
		}
		else {			
			memcpy((char *) to_fill + filled, _buffer + index, _sizeof_buffer - index);
			bytes_remaining -= _sizeof_buffer - index;
			filled += _sizeof_buffer - index;
			index = 0;
		}
	}
}

uint32_t RingBuffer::get_index() {
	return _index;
}

void* RingBuffer::get_at_index(uint32_t index) {
  return _buffer + (_index + index * _sizeof_data) % _sizeof_buffer;
}
