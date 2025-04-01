#pragma once

#include <stdio.h>

namespace vanjee
{
namespace lidar
{
        inline void hexdump(const uint8_t *data, size_t size, const char *desc = NULL)
        {
            printf("\n---------------%s(size:%d)------------------", (desc ? desc : ""), (int)size);

            for (size_t i = 0; i < size; i++)
            {
                if (i % 16 == 0)
                    printf("\n");
                printf("%02x ", data[i]);
            }
            printf("\n---------------------------------\n");
        }
} 
} 
