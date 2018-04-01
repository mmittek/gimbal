#include "utils.h"



void utils_get_ms(long unsigned int *timestamp) {
	*timestamp = HAL_GetTick();
}
