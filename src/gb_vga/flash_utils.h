// Thanks to Jan Cumps from Element14 post:
// https://community.element14.com/products/raspberry-pi/b/blog/posts/raspberry-pico-c-sdk-reserve-a-flash-memory-block-for-persistent-storage


#include <stdint.h>

inline uint32_t *getAddressPersistent() {
extern uint32_t ADDR_PERSISTENT[];
    return ADDR_PERSISTENT;
}