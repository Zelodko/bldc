#ifndef FOC_SAMPLE_H_
#define FOC_SAMPLE_H_

#include <stdbool.h>
#include <stdint.h>

void foc_sample_init(void);
void foc_sample_capture(uint32_t dma_flags);
bool foc_sample_get_voltage(int voltage[3], bool is_second_motor);

#endif
