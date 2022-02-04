#pragma once



#include "gd32vf103.h"

#define LIO_BT_BUFFER_SIZE 1024


void lio_init_bt();
size_t lio_read_bt(char* buffer, uint32_t size);
size_t lio_send_bt(char* message, uint32_t size);