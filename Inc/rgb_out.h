#ifndef __RGB_OUT_H
#define __RGB_OUT_H
#define RGB_OUT(GPIOx, GPIO_PIN,STATUS) HAL_GPIO_WritePin(GPIOx,GPIO_PIN,STATUS);

#define RGB_A(STATUS) RGB_OUT(GPIOA, GPIO_PIN_12, STATUS)
#define RGB_B(STATUS) RGB_OUT(GPIOA, GPIO_PIN_11, STATUS)
#define RGB_C(STATUS) RGB_OUT(GPIOA, GPIO_PIN_10, STATUS)
#define RGB_D(STATUS) RGB_OUT(GPIOA, GPIO_PIN_9, STATUS)

#define RGB_R1(STATUS) RGB_OUT(GPIOB, GPIO_PIN_3, STATUS)
#define RGB_G1(STATUS) RGB_OUT(GPIOB, GPIO_PIN_4, STATUS)
#define RGB_B1(STATUS) RGB_OUT(GPIOB, GPIO_PIN_5, STATUS)

#define RGB_R2(STATUS) RGB_OUT(GPIOB, GPIO_PIN_7, STATUS)
#define RGB_G2(STATUS) RGB_OUT(GPIOB, GPIO_PIN_8, STATUS)
#define RGB_B2(STATUS) RGB_OUT(GPIOB, GPIO_PIN_9, STATUS)

#define RGB_CLK(STATUS) RGB_OUT(GPIOB, GPIO_PIN_6, STATUS)
#define RGB_OE(STATUS) RGB_OUT(GPIOA, GPIO_PIN_8, STATUS_SWAP(STATUS))
#define RGB_LAT(STATUS) RGB_OUT(GPIOA, GPIO_PIN_15, STATUS)

#define TEST(STATUS) RGB_OUT(GPIOC, GPIO_PIN_13, STATUS_SWAP(STATUS))

#define NROWS 32 
#define NCOLS 128
#define NBIT 4

#define NDATA 3 * NCOLS * NROWS / 2

#define OUT_ON GPIO_PIN_SET
#define OUT_OFF GPIO_PIN_RESET
#define STATUS_SWAP(STATUS) ((STATUS == OUT_ON) ? OUT_OFF : OUT_ON)

#endif /* __RGB_OUT_H */