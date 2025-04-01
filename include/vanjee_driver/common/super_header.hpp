#pragma once


typedef unsigned char             uint8;
typedef signed char               int8;
typedef unsigned short            uint16;
typedef signed short              int16;
typedef unsigned int              uint32;
typedef signed int                int32;
typedef unsigned long long        uint64;
typedef signed long long          int64;
typedef float                     float32;
typedef double                    float64;

#include <vector>
#include <memory>
using ByteVector = std::vector<uint8>;
using ByteVectorPtr = std::shared_ptr<std::vector<uint8>>;

template <typename T>
using TPtr = typename std::shared_ptr<T>;