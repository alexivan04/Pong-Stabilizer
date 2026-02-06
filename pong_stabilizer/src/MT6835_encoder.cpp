#include "MT6835_encoder.h"
#include "core_pins.h"
#include "usb_serial.h"
#include <cstdint>

const int8_t MT6835::LOOKUP[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

MT6835::MT6835(SPIClass *spi, uint8_t cs, uint8_t cal_en, uint8_t a_pin, uint8_t b_pin) 
    : _spi(spi), _cs(cs), _cal_en(cal_en), _a_pin(a_pin), _b_pin(b_pin) {}

int MT6835::begin() {
    pinMode(_cs, OUTPUT);
    digitalWriteFast(_cs, HIGH);
    pinMode(_cal_en, OUTPUT);
    digitalWriteFast(_cal_en, LOW);

    pinMode(_a_pin, INPUT_PULLUP);
    pinMode(_b_pin, INPUT_PULLUP);

    _spi->begin();

    _enc_value = (digitalReadFast(_a_pin) << 1) | digitalReadFast(_b_pin);

    // set ABZ rate to match ENCODER_PPR
    uint16_t abz_rez_code = ENCODER_PPR - 1;
    if(getABZRez() != abz_rez_code) {
        Serial.println("[MT6835] writing ABZ Res to EEPROM");
        setABZRez(abz_rez_code);
        programEEPROM();
        return -1;
    }
    else Serial.println("[MT6835] resolution already set");
    return 0;
}

// High-speed ISR handler
void MT6835::handleISR() {
    _enc_value = _enc_value << 2;
    _enc_value = _enc_value | ((digitalReadFast(_a_pin) << 1 | digitalReadFast(_b_pin)));
    _raw_pulses = _raw_pulses + LOOKUP[_enc_value & 0b1111];
}

float MT6835::getAngle() const {
    return (STEPS_TO_DEG(_raw_pulses));
}

uint8_t MT6835::readRegister(uint16_t addr) {
    _spi->beginTransaction(_settings);
    digitalWriteFast(_cs, LOW);
    delay(10);
    _spi->transfer16(OP_READ | (addr & 0x0FFF));
    uint8_t val = _spi->transfer(0);
    delay(10);
    digitalWriteFast(_cs, HIGH);
    delay(10);
    _spi->endTransaction();
    return val;
}

void MT6835::writeRegister(uint16_t addr, uint8_t data) {
    _spi->beginTransaction(_settings);
    digitalWriteFast(_cs, LOW);
    delay(10);
    _spi->transfer16(OP_WRITE | (addr & 0x0FFF));
    _spi->transfer(data);
    delay(10);
    digitalWriteFast(_cs, HIGH);
    delay(10);
    _spi->endTransaction();
}

void MT6835::programEEPROM(){
   _spi->beginTransaction(_settings);
    digitalWriteFast(_cs, LOW);
    delay(10);
    _spi->transfer16(OP_EEPROM_PROGRAM);
    uint8_t ack = _spi->transfer(0);
    delay(10);
    digitalWriteFast(_cs, HIGH);
    delay(10);

    if (ack == 0x55) {
        Serial.println("[MT6835] EEPROM programmed, waiting 6s");
        delay(6500);
        Serial.println("[MT6835] turn off system");
    }
    else Serial.println("[MT6835] ERROR programming EEPROM");
    _spi->endTransaction();
}

void MT6835::checkHealth() {
    uint8_t reg005 = readRegister(0x005);

    if (reg005 & 0x01) Serial.println("[MT6835] WARNING: rotation over speed");
    if (reg005 & 0x02) Serial.println("[MT6835] WARNING: weak magnetic field");
    if (reg005 & 0x04) Serial.println("[MT6835] WARNING: undervoltage");
}

void MT6835::setFrequencyRange(uint8_t autocal_freq) {
    uint8_t reg0E = readRegister(0x00E);
    reg0E = (reg0E & 0x8F) | ((autocal_freq & 0x07) << 4);
    writeRegister(0x00E, reg0E);

    Serial.println("[MT6835] frequency range set");

}

uint8_t MT6835::getFrequencyRange() {
    uint8_t reg0E = readRegister(0x00E);
    uint8_t autocal_freq = (reg0E >> 4) & 0x07;
    return autocal_freq;
}

void MT6835::setABZRez(uint16_t rez) {
    rez &= 0x3FFF;

    uint8_t reg07 = (rez >> 6) & 0xFF;   // ABZ_RES[13:6]
    uint8_t reg08 = readRegister(0x008); // preserve other bits
    reg08 = (reg08 & 0x03) | ((rez & 0x3F) << 2); // ABZ_RES[5:0]

    writeRegister(0x007, reg07);
    writeRegister(0x008, reg08);
}

uint16_t MT6835::getABZRez() {
    uint8_t reg07 = readRegister(0x007);
    uint8_t reg08 = readRegister(0x008);
    uint16_t abz_rez = ((uint16_t)reg07 << 6) | ((reg08 >> 2) & 0x3F);

    return abz_rez;
}

bool MT6835::autoCalibrate(uint32_t (*getRevs)()) {
    digitalWriteFast(_cal_en, HIGH);

    uint32_t start_time = millis();
    uint8_t cal_status = 0;

    while (true) {
        this->checkHealth();
        uint8_t reg113 = readRegister(0x113);
        cal_status = (reg113 >> 6) & 0x03;

        if (cal_status == 0b11) {
            Serial.println("[MT6835] SUCCESS, EEPROM written, waiting 6s...");
            delay(6000);
            digitalWriteFast(_cal_en, LOW);
            return true;
        } 

        else if (cal_status == 0b10) {
            Serial.println("[MT6835] FAILED");
            digitalWriteFast(_cal_en, LOW);
            return false;
        }

        if (millis() - start_time > 60000) {
            Serial.println("[MT6835] TIMEOUT");
            digitalWriteFast(_cal_en, LOW);
            return false;
        }

        if (getRevs) {
            static uint32_t last_m = 0;
            if (millis() - last_m > 1000) {
                Serial.printf("[MT6835] calibrating, revs done: %lu\n", getRevs());
                last_m = millis();
            }
        }
        delay(25);
    }
}
