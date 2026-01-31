#include <Arduino.h>
#include <SPI.h>

#define T_L 100 //time between falling edges of CSN and SCK

struct MT6385_encoder {
    SPIClass *spi_controller;
    uint8_t cs_pin;
    uint8_t config_pin;

    uint8_t   ;
}
