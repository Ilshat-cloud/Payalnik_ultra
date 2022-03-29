#include "MAX31855.h"

void MAX31855_convert_buf(MAX31855_Sensor *sensor)
{
    if (sizeof(sensor->Buf)>3)   //---Bufer is not empty
    {
      int16_t temp; 
      temp=(sensor->Buf[2]&0x00FF)|((sensor->Buf[3]<<8)&0xFF00);
      sensor->OC=temp&0x0001;
      sensor->SCG=(temp>>1)&0x0001;
      sensor->SCV=(temp>>2)&0x0001;
      temp>>=4;
      if (temp&0x1000)
      {
        temp=0xF800|(temp&0x7FF);
      }
      sensor->t_int=temp*0.0625;
      temp=(sensor->Buf[0]&0x00FF)|((sensor->Buf[1]<<8)&0xFF00);
      temp>>=2;
      if (temp&0x2000)
      {
        temp=0xE000|(temp&0x1FFF);
      }
      sensor->t_termocoupe=temp*0.25;  
    }

}