#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void Encoder_Init(void);
void Encoder_ResetCounts(void);

int32_t Encoder_GetLeftCount(void);
int32_t Encoder_GetRightCount(void);

#endif
