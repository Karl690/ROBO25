#pragma once
#include <stdint.h> 
extern uint32_t PnPFeederValue;
extern uint16_t PnPResetTimer;//spi3 valve timer dwell time
extern uint16_t PNPSPIData;//spi3 valve control data word;


void LatchPnPData();
void PnP_SetValves();
void PnP_TurnOffAllValves();
void SendPNPSPIDataToSpi2(uint16_t DataToSend);
void Init_SPI2(void);
void Init_SPI3(void);

