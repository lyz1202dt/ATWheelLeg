#ifndef BASELIB_COMM_PACK_H_
#define BASELIB_COMM_PACK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#pragma pack(push, 1)

typedef struct {
    uint32_t cmd;
    uint32_t size;
    uint8_t data[];
} Command;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif // BASELIB_COMM_PACK_H_
