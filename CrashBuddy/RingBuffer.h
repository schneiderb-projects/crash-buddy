#ifndef RingBuffer_h
#define RingBuffer_h
#include <Arduino.h>
class RingBuffer {
  private:
    char* _buffer;
    uint32_t _index;
    uint8_t _sizeof_data;
    uint32_t _sizeof_buffer;
  public:
    RingBuffer(void* buffer, uint32_t sizeof_buffer, uint8_t sizeof_data);
    void push(void* data); // push new data to the ring buffer. new data must be of size "this->sizeof_data"
    void pop(uint32_t offset, uint32_t size, void* to_fill); // pop "size" many elements starting at ("this->index" + "offset") and place them in "to_fill"
    uint32_t get_index(); //get current location of index
    void* get_at_index(uint32_t index); //gets a pointer to the item at "_index + index * _sizeof_data"
};
#endif
