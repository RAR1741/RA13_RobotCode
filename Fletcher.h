#ifndef FLETCHER_H__
#define FLETCHER_H__

#include <cstddef>
#include <cstdio>


uint16_t Fletcher16( FILE *fp, size_t * bytes_read );

#endif
