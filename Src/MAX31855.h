//----------------перед использованием настроить и вернуть данные с spi-------------------------------------//
/*		hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
		hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
		hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
		hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;     */
//---------------------------------------------------------------------------------------------------------//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "main.h"

#ifndef MAX31855
#define MAX31855


typedef struct {
	 float t_int;    /*!< internal temperature*/
	 float t_termocoupe;   /*!< termocoupe temperature */
	 uint8_t *Buf; /*!< Pointer to recive data */
	 uint8_t SCV;	/*!< Short to VCC */
	 uint8_t SCG;	/*!< Short to GND */
	 uint8_t OC;	/*!< Open Circuit */
} MAX31855_Sensor;

void MAX31855_convert_buf(MAX31855_Sensor *sensor);


#endif