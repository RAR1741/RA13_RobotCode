#include "fletcher.h"

/**
 * @brief Computes checksum of data
 */

uint16_t Fletcher16( FILE *fp, size_t * bytes_read )
{
	uint16_t sum1 = 0;
	uint16_t sum2 = 0;
	*bytes_read = 0;
	int data = fgetc(fp);
	
	while (!feof(fp))
   {
	  ++(*bytes_read);
      sum1 = (sum1 + static_cast<uint8_t>(data)) % 255;
      sum2 = (sum2 + sum1) % 255;
      data = fgetc(fp);
   } 

   return (sum2 << 8) | sum1;
}

