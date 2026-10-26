#ifndef MT6835_H
#define MT6835_H

#include <Arduino.h>
#include <SPI.h>
#include <cstdint>

#define ENCODER_PPR            16384
#define ENCODER_PPR_MULTIPLIER 4
#define STEPS_TO_DEG(steps)    ((steps) * (360.0f / (ENCODER_PPR * ENCODER_PPR_MULTIPLIER)))
#define DEG_TO_STEPS(deg)      ((deg) * ((ENCODER_PPR * ENCODER_PPR_MULTIPLIER) / 360.0f))

#define OP_READ               0x3000
#define OP_WRITE              0x6000
#define OP_EEPROM_PROGRAM     0xC000
#define OP_AUTO_ZERO_POS      0x5000

#define ZERO_POS_FINESSE      0.088f

class MT6835 {
public:

    MT6835(SPIClass *spi, uint8_t cs, uint8_t cal_en, uint8_t a_pin, uint8_t b_pin);

    int begin();

    void handleISR();
    int32_t getRawPulses() const { return _raw_pulses; }
    float getAngle() const;
    void resetPulses() { _raw_pulses = 0; }

    void setFrequencyRange(uint8_t autocal_freq);
    uint8_t getFrequencyRange();
    void setABZRez(uint16_t rez);
    uint16_t getABZRez();
    void setZeroPos();
    uint16_t getZeroPos();

    bool autoCalibrate(uint32_t (*getRevs)() = nullptr);
    void checkHealth();

    uint8_t readRegister(uint16_t addr);
    void writeRegister(uint16_t addr, uint8_t data);
    void programEEPROM();

    void debugHiddenBits();

private:
    SPIClass *_spi;
    uint8_t _cs, _cal_en, _a_pin, _b_pin;

    volatile int32_t _raw_pulses = 0;
    volatile uint8_t _enc_value = 0;
    static const int8_t LOOKUP[];

    const SPISettings _settings = SPISettings(4000000, MSBFIRST, SPI_MODE3);
};

#endif
