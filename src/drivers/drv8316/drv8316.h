#pragma once

#include <SPI.h>
#include <drivers/BLDCDriver3PWM.h>
#include <drivers/BLDCDriver6PWM.h>

#include "./drv8316_registers.h"
#include "Arduino.h"

enum DRV8316_PWMMode {
  PWM6_Mode = 0b00,
  PWM6_CurrentLimit_Mode = 0b01,
  PWM3_Mode = 0b10,
  PWM3_CurrentLimit_Mode = 0b11
};

enum DRV8316_SDOMode { SDOMode_OpenDrain = 0b0, SDOMode_PushPull = 0b1 };

enum DRV8316_Slew {
  Slew_25Vus = 0b00,
  Slew_50Vus = 0b01,
  Slew_150Vus = 0b10,
  Slew_200Vus = 0b11
};

enum DRV8316_OVP { OVP_SEL_32V = 0b0, OVP_SEL_20V = 0b1 };

enum DRV8316_PWM100DUTY { FREQ_20KHz = 0b0, FREQ_40KHz = 0b1 };

enum DRV8316_OCPMode {
  Latched_Fault = 0b00,
  AutoRetry_Fault = 0b01,
  ReportOnly = 0b10,
  NoAction = 0b11
};

enum DRV8316_OCPLevel { Curr_16A = 0b0, Curr_24A = 0b1 };

enum DRV8316_OCPRetry { Retry5ms = 0b0, Retry500ms = 0b1 };

enum DRV8316_OCPDeglitch {
  Deglitch_0us2 = 0b00,
  Deglitch_0us6 = 0b01,
  Deglitch_1us1 = 0b10,
  Deglitch_1us6 = 0b11
};

enum DRV8316_CSAGain {
  Gain_0V15 = 0b00,
  Gain_0V1875 = 0b01,
  Gain_0V25 = 0b10,
  Gain_0V375 = 0b11
};

enum DRV8316_Recirculation {
  BrakeMode = 0b00,  // FETs
  CoastMode = 0b01   // Diodes
};

enum DRV8316_BuckVoltage {
  VB_3V3 = 0b00,
  VB_5V = 0b01,
  VB_4V = 0b10,
  VB_5V7 = 0b11
};

enum DRV8316_BuckCurrentLimit { Limit_600mA = 0b00, Limit_150mA = 0b01 };

enum DRV8316_DelayTarget {
  Delay_0us = 0x0,
  Delay_0us4 = 0x1,
  Delay_0us6 = 0x2,
  Delay_0us8 = 0x3,
  Delay_1us = 0x4,
  Delay_1us2 = 0x5,
  Delay_1us4 = 0x6,
  Delay_1us6 = 0x7,
  Delay_1us8 = 0x8,
  Delay_2us = 0x9,
  Delay_2us2 = 0xA,
  Delay_2us4 = 0xB,
  Delay_2us6 = 0xC,
  Delay_2us8 = 0xD,
  Delay_3us = 0xE,
  Delay_3us2 = 0xF
};

class DRV8316ICStatus {
 public:
  DRV8316ICStatus(IC_Status status) : status(status) {};
  ~DRV8316ICStatus() {};

  bool isFault() const { return status.FAULT == 0b1; };
  bool isOverTemperature() const { return status.OT == 0b1; };
  bool isOverCurrent() const { return status.OCP == 0b1; };
  bool isOverVoltage() const { return status.OVP == 0b1; };
  bool isSPIError() const { return status.SPI_FLT == 0b1; };
  bool isBuckError() const { return status.BK_FLT == 0b1; };
  bool isPowerOnReset() const { return status.NPOR == 0b1; };

  IC_Status status;
};

class DRV8316Status1 {
 public:
  DRV8316Status1(Status__1 status1) : status1(status1) {};
  ~DRV8316Status1() {};

  bool isOverCurrent_Ah() const { return status1.OCP_HA == 0b1; };
  bool isOverCurrent_Al() const { return status1.OCP_LA == 0b1; };
  bool isOverCurrent_Bh() const { return status1.OCP_HB == 0b1; };
  bool isOverCurrent_Bl() const { return status1.OCP_LB == 0b1; };
  bool isOverCurrent_Ch() const { return status1.OCP_HC == 0b1; };
  bool isOverCurrent_Cl() const { return status1.OCP_LC == 0b1; };
  bool isOverTemperatureShutdown() const { return status1.OTS == 0b1; };
  bool isOverTemperatureWarning() const { return status1.OTW == 0b1; };

  Status__1 status1;
};

class DRV8316Status2 {
 public:
  DRV8316Status2(Status__2 status2) : status2(status2) {};
  ~DRV8316Status2() {};

  bool isOneTimeProgrammingError() const { return status2.OTP_ERR == 0b1; };
  bool isBuckOverCurrent() const { return status2.BUCK_OCP == 0b1; };
  bool isBuckUnderVoltage() const { return status2.BUCK_UV == 0b1; };
  bool isChargePumpUnderVoltage() const { return status2.VCP_UV == 0b1; };
  bool isSPIParityError() const { return status2.SPI_PARITY == 0b1; };
  bool isSPIClockFramingError() const { return status2.SPI_SCLK_FLT == 0b1; };
  bool isSPIAddressError() const { return status2.SPI_ADDR_FLT == 0b1; };

  Status__2 status2;
};

class DRV8316Status : public DRV8316ICStatus,
                      public DRV8316Status1,
                      public DRV8316Status2 {
 public:
  DRV8316Status(IC_Status status, Status__1 status1, Status__2 status2)
      : DRV8316ICStatus(status),
        DRV8316Status1(status1),
        DRV8316Status2(status2) {};
  ~DRV8316Status() {};

  void print() const;
};

template <class PIN>
class DRV8316Driver {
 public:
  DRV8316Driver(PIN cs, bool currentLimit = false, int nFault = NOT_SET)
      : currentLimit(currentLimit),
        cs(cs),
        nFault(nFault),
        spi(&SPI),
        settings(1000000, MSBFIRST, SPI_MODE1) {};
  virtual ~DRV8316Driver() {};

  virtual void init(SPIClass* _spi = &SPI);

  void clearFault();  // TODO check for fault condition methods

  DRV8316Status getStatus();

  bool isRegistersLocked();
  void setRegistersLocked(bool lock);

  DRV8316_PWMMode getPWMMode();
  void setPWMMode(DRV8316_PWMMode pwmMode);

  DRV8316_Slew getSlew();
  void setSlew(DRV8316_Slew slewRate);

  DRV8316_SDOMode getSDOMode();
  void setSDOMode(DRV8316_SDOMode sdoMode);

  bool isOvertemperatureReporting();
  void setOvertemperatureReporting(bool reportFault);

  bool isSPIFaultReporting();
  void setSPIFaultReporting(bool reportFault);

  bool isOvervoltageProtection();
  void setOvervoltageProtection(bool enabled);

  DRV8316_OVP getOvervoltageLevel();
  void setOvervoltageLevel(DRV8316_OVP voltage);

  DRV8316_PWM100DUTY getPWM100Frequency();
  void setPWM100Frequency(DRV8316_PWM100DUTY freq);

  DRV8316_OCPMode getOCPMode();
  void setOCPMode(DRV8316_OCPMode ocpMode);

  DRV8316_OCPLevel getOCPLevel();
  void setOCPLevel(DRV8316_OCPLevel amps);

  DRV8316_OCPRetry getOCPRetryTime();
  void setOCPRetryTime(DRV8316_OCPRetry ms);

  DRV8316_OCPDeglitch getOCPDeglitchTime();
  void setOCPDeglitchTime(DRV8316_OCPDeglitch ms);

  bool isOCPClearInPWMCycleChange();
  void setOCPClearInPWMCycleChange(bool enable);

  bool isDriverOffEnabled();
  void setDriverOffEnabled(bool enabled);

  DRV8316_CSAGain getCurrentSenseGain();
  void setCurrentSenseGain(DRV8316_CSAGain gain);

  bool isActiveSynchronousRectificationEnabled();
  void setActiveSynchronousRectificationEnabled(bool enabled);

  bool isActiveAsynchronousRectificationEnabled();
  void setActiveAsynchronousRectificationEnabled(bool enabled);

  DRV8316_Recirculation getRecirculationMode();
  void setRecirculationMode(DRV8316_Recirculation recirculationMode);

  bool isBuckEnabled();
  void setBuckEnabled(bool enabled);

  DRV8316_BuckVoltage getBuckVoltage();
  void setBuckVoltage(DRV8316_BuckVoltage volts);

  DRV8316_BuckCurrentLimit getBuckCurrentLimit();
  void setBuckCurrentLimit(DRV8316_BuckCurrentLimit mamps);

  bool isBuckPowerSequencingEnabled();
  void setBuckPowerSequencingEnabled(bool enabled);

  DRV8316_DelayTarget getDelayTarget();
  void setDelayTarget(DRV8316_DelayTarget us);

  bool isDelayCompensationEnabled();
  void setDelayCompensationEnabled(bool enabled);

 private:
  uint16_t readSPI(uint8_t addr);
  uint16_t writeSPI(uint8_t addr, uint8_t data);
  bool getParity(uint16_t data);

  bool currentLimit;
  PIN cs;
  int nFault;
  SPIClass* spi;
  SPISettings settings;
};

template <class PIN>
class DRV8316Driver3PWM : public DRV8316Driver<PIN>, public BLDCDriver3PWM {
 public:
  DRV8316Driver3PWM(int phA, int phB, int phC, PIN cs,
      bool currentLimit = false, int en = NOT_SET, int nFault = NOT_SET)
      : DRV8316Driver<PIN>(cs, currentLimit, nFault),
        BLDCDriver3PWM(phA, phB, phC, en) {
    enable_active_high = false;
  };
  virtual ~DRV8316Driver3PWM() {};

  virtual void init(SPIClass* _spi = &SPI) override;
};

template <class PIN>
class DRV8316Driver6PWM : public DRV8316Driver<PIN>, public BLDCDriver6PWM {
 public:
  DRV8316Driver6PWM(int phA_h, int phA_l, int phB_h, int phB_l, int phC_h,
      int phC_l, PIN cs, bool currentLimit = false, int en = NOT_SET,
      int nFault = NOT_SET)
      : DRV8316Driver<PIN>(cs, currentLimit, nFault),
        BLDCDriver6PWM(phA_h, phA_l, phB_h, phB_l, phC_h, phC_l, en) {
    enable_active_high = false;
  };
  virtual ~DRV8316Driver6PWM() {};

  virtual void init(SPIClass* _spi = &SPI) override;
};

//==============================================================================

#include "./drv8316.h"

template <class PIN>
void DRV8316Driver3PWM<PIN>::init(SPIClass* _spi) {
  DRV8316Driver<PIN>::init(_spi);
  DRV8316Driver<PIN>::setRegistersLocked(false);
  delayMicroseconds(1);
  DRV8316Driver<PIN>::setPWMMode(DRV8316_PWMMode::PWM3_Mode);
  BLDCDriver3PWM::init();
};

template <class PIN>
void DRV8316Driver6PWM<PIN>::init(SPIClass* _spi) {
  DRV8316Driver<PIN>::init(_spi);
  DRV8316Driver<PIN>::setRegistersLocked(false);
  delayMicroseconds(1);
  DRV8316Driver<PIN>::setPWMMode(
      DRV8316_PWMMode::PWM6_Mode);  // default mode is 6-PWM
  BLDCDriver6PWM::init();
};

/*
 * SPI setup:
 *
 *  capture on falling, propagate on rising =
 *  MSB first
 *
 * 	16 bit words
 * 	 outgoing: R/W:1 addr:6 parity:1 data:8
 * 	 incoming: status:8 data:8
 *
 * 	 on reads, incoming data is content of register being read
 * 	 on writes, incomnig data is content of register being written
 *
 *
 */

template <class PIN>
void DRV8316Driver<PIN>::init(SPIClass* _spi) {
  // TODO make SPI speed configurable
  spi = _spi;
  settings = SPISettings(1000000, MSBFIRST, SPI_MODE1);

  // setup pins
  cs.pinMode(OUTPUT);
  cs.digitalWrite(HIGH);  // switch off

  // SPI has an internal SPI-device counter, it is possible to call "begin()"
  // from different devices
  spi->begin();

  if (_isset(nFault)) {
    pinMode(nFault, INPUT);
    // TODO add interrupt handler on the nFault pin if configured
    // add configuration for how to handle faults... idea: interrupt handler
    // calls a callback, depending on the type of fault consider what would be a
    // useful configuration in practice? What do we want to do on a fault, e.g.
    // over-temperature for example?

    // attachInterrupt(digitalPinToInterrupt(nFault), handleInterrupt,
    // PinStatus::FALLING);
  }
};

template <class PIN>
bool DRV8316Driver<PIN>::getParity(uint16_t data) {
  // PARITY = XNOR(CMD, A5..A0, D7..D0)
  uint8_t par = 0;
  for (int i = 0; i < 16; i++) {
    if (((data) >> i) & 0x0001) par += 1;
  }
  return (par & 0x01) == 0x01;  // even number of bits means true
}

template <class PIN>
uint16_t DRV8316Driver<PIN>::readSPI(uint8_t addr) {
  cs.digitalWrite(0);
  spi->beginTransaction(settings);
  uint16_t data = (((addr << 1) | 0x80) << 8) | 0x0000;
  if (getParity(data)) data |= 0x0100;
  uint16_t result = spi->transfer16(data);
  spi->endTransaction();
  cs.digitalWrite(1);
  //	Serial.print("SPI Read Result: ");
  //	Serial.print(data, HEX);
  //	Serial.print(" -> ");
  //	Serial.println(result, HEX);
  return result;
}

template <class PIN>
uint16_t DRV8316Driver<PIN>::writeSPI(uint8_t addr, uint8_t value) {
  cs.digitalWrite(0);
  spi->beginTransaction(settings);
  uint16_t data = ((addr << 1) << 8) | value;
  if (getParity(data)) data |= 0x0100;
  uint16_t result = spi->transfer16(data);
  spi->endTransaction();
  cs.digitalWrite(1);
  //	Serial.print("SPI Write Result: ");
  //	Serial.print(data, HEX);
  //	Serial.print(" -> ");
  //	Serial.println(result, HEX);
  return result;
}

template <class PIN>
DRV8316Status DRV8316Driver<PIN>::getStatus() {
  IC_Status data;
  Status__1 data1;
  Status__2 data2;
  uint16_t result = readSPI(IC_Status_ADDR);
  data.reg = (result & 0x00FF);
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = readSPI(Status__1_ADDR);
  data1.reg = (result & 0x00FF);
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = readSPI(Status__2_ADDR);
  data2.reg = (result & 0x00FF);
  return DRV8316Status(data, data1, data2);
}

template <class PIN>
void DRV8316Driver<PIN>::clearFault() {
  uint16_t result = readSPI(Control__2_ADDR);
  Control__2 data;
  data.reg = (result & 0x00FF);
  data.CLR_FLT |= 1;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__2_ADDR, data.reg);
};

template <class PIN>
bool DRV8316Driver<PIN>::isRegistersLocked() {
  uint16_t result = readSPI(Control__1_ADDR);
  Control__1 data;
  data.reg = (result & 0x00FF);
  return data.REG_LOCK == REG_LOCK_LOCK;
}

template <class PIN>
void DRV8316Driver<PIN>::setRegistersLocked(bool lock) {
  uint16_t result = readSPI(Control__1_ADDR);
  Control__1 data;
  data.reg = (result & 0x00FF);
  data.REG_LOCK = lock ? REG_LOCK_LOCK : REG_LOCK_UNLOCK;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__1_ADDR, data.reg);
}

template <class PIN>
DRV8316_PWMMode DRV8316Driver<PIN>::getPWMMode() {
  uint16_t result = readSPI(Control__2_ADDR);
  Control__2 data;
  data.reg = (result & 0x00FF);
  return (DRV8316_PWMMode)data.PWM_MODE;
};

template <class PIN>
void DRV8316Driver<PIN>::setPWMMode(DRV8316_PWMMode pwmMode) {
  uint16_t result = readSPI(Control__2_ADDR);
  Control__2 data;
  data.reg = (result & 0x00FF);
  data.PWM_MODE = pwmMode;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__2_ADDR, data.reg);
};

template <class PIN>
DRV8316_Slew DRV8316Driver<PIN>::getSlew() {
  uint16_t result = readSPI(Control__2_ADDR);
  Control__2 data;
  data.reg = (result & 0x00FF);
  return (DRV8316_Slew)data.SLEW;
};

template <class PIN>
void DRV8316Driver<PIN>::setSlew(DRV8316_Slew slewRate) {
  uint16_t result = readSPI(Control__2_ADDR);
  Control__2 data;
  data.reg = (result & 0x00FF);
  data.SLEW = slewRate;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__2_ADDR, data.reg);
};

template <class PIN>
DRV8316_SDOMode DRV8316Driver<PIN>::getSDOMode() {
  uint16_t result = readSPI(Control__2_ADDR);
  Control__2 data;
  data.reg = (result & 0x00FF);
  return (DRV8316_SDOMode)data.SDO_MODE;
};

template <class PIN>
void DRV8316Driver<PIN>::setSDOMode(DRV8316_SDOMode sdoMode) {
  uint16_t result = readSPI(Control__2_ADDR);
  Control__2 data;
  data.reg = (result & 0x00FF);
  data.SDO_MODE = sdoMode;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__2_ADDR, data.reg);
};

template <class PIN>
bool DRV8316Driver<PIN>::isOvertemperatureReporting() {
  uint16_t result = readSPI(Control__3_ADDR);
  Control__3 data;
  data.reg = (result & 0x00FF);
  return data.OTW_REP == OTW_REP_ENABLE;
};

template <class PIN>
void DRV8316Driver<PIN>::setOvertemperatureReporting(bool reportFault) {
  uint16_t result = readSPI(Control__3_ADDR);
  Control__3 data;
  data.reg = (result & 0x00FF);
  data.OTW_REP = reportFault ? OTW_REP_ENABLE : OTW_REP_DISABLE;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__3_ADDR, data.reg);
};

template <class PIN>
bool DRV8316Driver<PIN>::isSPIFaultReporting() {
  uint16_t result = readSPI(Control__3_ADDR);
  Control__3 data;
  data.reg = (result & 0x00FF);
  return data.SPI_FLT_REP == SPI_FLT_REP_ENABLE;
}

template <class PIN>
void DRV8316Driver<PIN>::setSPIFaultReporting(bool reportFault) {
  uint16_t result = readSPI(Control__3_ADDR);
  Control__3 data;
  data.reg = (result & 0x00FF);
  data.SPI_FLT_REP = reportFault ? SPI_FLT_REP_ENABLE : SPI_FLT_REP_DISABLE;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__3_ADDR, data.reg);
}

template <class PIN>
bool DRV8316Driver<PIN>::isOvervoltageProtection() {
  uint16_t result = readSPI(Control__3_ADDR);
  Control__3 data;
  data.reg = (result & 0x00FF);
  return data.OVP_EN == OVP_EN_ENABLE;
};
template <class PIN>
void DRV8316Driver<PIN>::setOvervoltageProtection(bool enabled) {
  uint16_t result = readSPI(Control__3_ADDR);
  Control__3 data;
  data.reg = (result & 0x00FF);
  data.OVP_EN = enabled ? OVP_EN_ENABLE : OVP_EN_DISABLE;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__3_ADDR, data.reg);
};

template <class PIN>
DRV8316_OVP DRV8316Driver<PIN>::getOvervoltageLevel() {
  uint16_t result = readSPI(Control__3_ADDR);
  Control__3 data;
  data.reg = (result & 0x00FF);
  return (DRV8316_OVP)data.OVP_SEL;
};
template <class PIN>
void DRV8316Driver<PIN>::setOvervoltageLevel(DRV8316_OVP voltage) {
  uint16_t result = readSPI(Control__3_ADDR);
  Control__3 data;
  data.reg = (result & 0x00FF);
  data.OVP_SEL = voltage;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__3_ADDR, data.reg);
};

template <class PIN>
DRV8316_PWM100DUTY DRV8316Driver<PIN>::getPWM100Frequency() {
  uint16_t result = readSPI(Control__3_ADDR);
  Control__3 data;
  data.reg = (result & 0x00FF);
  return (DRV8316_PWM100DUTY)data.PWM_100_DUTY_SEL;
};
template <class PIN>
void DRV8316Driver<PIN>::setPWM100Frequency(DRV8316_PWM100DUTY freq) {
  uint16_t result = readSPI(Control__3_ADDR);
  Control__3 data;
  data.reg = (result & 0x00FF);
  data.PWM_100_DUTY_SEL = freq;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__3_ADDR, data.reg);
};
template <class PIN>
DRV8316_OCPMode DRV8316Driver<PIN>::getOCPMode() {
  uint16_t result = readSPI(Control__4_ADDR);
  Control__4 data;
  data.reg = (result & 0x00FF);
  return (DRV8316_OCPMode)data.OCP_MODE;
};
template <class PIN>
void DRV8316Driver<PIN>::setOCPMode(DRV8316_OCPMode ocpMode) {
  uint16_t result = readSPI(Control__4_ADDR);
  Control__4 data;
  data.reg = (result & 0x00FF);
  data.OCP_MODE = ocpMode;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__4_ADDR, data.reg);
};

template <class PIN>
DRV8316_OCPLevel DRV8316Driver<PIN>::getOCPLevel() {
  uint16_t result = readSPI(Control__4_ADDR);
  Control__4 data;
  data.reg = (result & 0x00FF);
  return (DRV8316_OCPLevel)data.OCP_LVL;
};
template <class PIN>
void DRV8316Driver<PIN>::setOCPLevel(DRV8316_OCPLevel amps) {
  uint16_t result = readSPI(Control__4_ADDR);
  Control__4 data;
  data.reg = (result & 0x00FF);
  data.OCP_LVL = amps;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__4_ADDR, data.reg);
};

template <class PIN>
DRV8316_OCPRetry DRV8316Driver<PIN>::getOCPRetryTime() {
  uint16_t result = readSPI(Control__4_ADDR);
  Control__4 data;
  data.reg = (result & 0x00FF);
  return (DRV8316_OCPRetry)data.OCP_RETRY;
};
template <class PIN>
void DRV8316Driver<PIN>::setOCPRetryTime(DRV8316_OCPRetry ms) {
  uint16_t result = readSPI(Control__4_ADDR);
  Control__4 data;
  data.reg = (result & 0x00FF);
  data.OCP_RETRY = ms;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__4_ADDR, data.reg);
};

template <class PIN>
DRV8316_OCPDeglitch DRV8316Driver<PIN>::getOCPDeglitchTime() {
  uint16_t result = readSPI(Control__4_ADDR);
  Control__4 data;
  data.reg = (result & 0x00FF);
  return (DRV8316_OCPDeglitch)data.OCP_DEG;
};
template <class PIN>
void DRV8316Driver<PIN>::setOCPDeglitchTime(DRV8316_OCPDeglitch ms) {
  uint16_t result = readSPI(Control__4_ADDR);
  Control__4 data;
  data.reg = (result & 0x00FF);
  data.OCP_DEG = ms;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__4_ADDR, data.reg);
};

template <class PIN>
bool DRV8316Driver<PIN>::isOCPClearInPWMCycleChange() {
  uint16_t result = readSPI(Control__4_ADDR);
  Control__4 data;
  data.reg = (result & 0x00FF);
  return data.OCP_CBC == OCP_CBC_ENABLE;
};
template <class PIN>
void DRV8316Driver<PIN>::setOCPClearInPWMCycleChange(bool enable) {
  uint16_t result = readSPI(Control__4_ADDR);
  Control__4 data;
  data.reg = (result & 0x00FF);
  data.OCP_CBC = enable ? OCP_CBC_ENABLE : OCP_CBC_DISABLE;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__4_ADDR, data.reg);
};

template <class PIN>
bool DRV8316Driver<PIN>::isDriverOffEnabled() {
  uint16_t result = readSPI(Control__4_ADDR);
  Control__4 data;
  data.reg = (result & 0x00FF);
  return data.DRV_OFF == DRV_OFF_ENABLE;
};
template <class PIN>
void DRV8316Driver<PIN>::setDriverOffEnabled(bool enabled) {
  uint16_t result = readSPI(Control__4_ADDR);
  Control__4 data;
  data.reg = (result & 0x00FF);
  data.DRV_OFF = enabled ? DRV_OFF_ENABLE : DRV_OFF_DISABLE;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__4_ADDR, data.reg);
};

template <class PIN>
DRV8316_CSAGain DRV8316Driver<PIN>::getCurrentSenseGain() {
  uint16_t result = readSPI(Control__5_ADDR);
  Control__5 data;
  data.reg = (result & 0x00FF);
  return (DRV8316_CSAGain)data.CSA_GAIN;
};
template <class PIN>
void DRV8316Driver<PIN>::setCurrentSenseGain(DRV8316_CSAGain gain) {
  uint16_t result = readSPI(Control__5_ADDR);
  Control__5 data;
  data.reg = (result & 0x00FF);
  data.CSA_GAIN = gain;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__5_ADDR, data.reg);
};

template <class PIN>
bool DRV8316Driver<PIN>::isActiveSynchronousRectificationEnabled() {
  uint16_t result = readSPI(Control__5_ADDR);
  Control__5 data;
  data.reg = (result & 0x00FF);
  return data.EN_ASR == EN_ASR_ENABLE;
};
template <class PIN>
void DRV8316Driver<PIN>::setActiveSynchronousRectificationEnabled(
    bool enabled) {
  uint16_t result = readSPI(Control__5_ADDR);
  Control__5 data;
  data.reg = (result & 0x00FF);
  data.EN_ASR = enabled ? EN_ASR_ENABLE : EN_ASR_DISABLE;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__5_ADDR, data.reg);
};

template <class PIN>
bool DRV8316Driver<PIN>::isActiveAsynchronousRectificationEnabled() {
  uint16_t result = readSPI(Control__5_ADDR);
  Control__5 data;
  data.reg = (result & 0x00FF);
  return data.EN_AAR == EN_AAR_ENABLE;
};
template <class PIN>
void DRV8316Driver<PIN>::setActiveAsynchronousRectificationEnabled(
    bool enabled) {
  uint16_t result = readSPI(Control__5_ADDR);
  Control__5 data;
  data.reg = (result & 0x00FF);
  data.EN_AAR = enabled ? EN_AAR_ENABLE : EN_AAR_DISABLE;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__5_ADDR, data.reg);
};

template <class PIN>
DRV8316_Recirculation DRV8316Driver<PIN>::getRecirculationMode() {
  uint16_t result = readSPI(Control__5_ADDR);
  Control__5 data;
  data.reg = (result & 0x00FF);
  return (DRV8316_Recirculation)data.ILIM_RECIR;
};
template <class PIN>
void DRV8316Driver<PIN>::setRecirculationMode(
    DRV8316_Recirculation recirculationMode) {
  uint16_t result = readSPI(Control__5_ADDR);
  Control__5 data;
  data.reg = (result & 0x00FF);
  data.ILIM_RECIR = recirculationMode;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__5_ADDR, data.reg);
};

template <class PIN>
bool DRV8316Driver<PIN>::isBuckEnabled() {
  uint16_t result = readSPI(Control__6_ADDR);
  Control__6 data;
  data.reg = (result & 0x00FF);
  return data.BUCK_DIS == BUCK_DIS_BUCK_ENABLE;
};
template <class PIN>
void DRV8316Driver<PIN>::setBuckEnabled(bool enabled) {
  uint16_t result = readSPI(Control__6_ADDR);
  Control__6 data;
  data.reg = (result & 0x00FF);
  data.BUCK_DIS = enabled ? BUCK_DIS_BUCK_ENABLE : BUCK_DIS_BUCK_DISABLE;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__6_ADDR, data.reg);
};

template <class PIN>
DRV8316_BuckVoltage DRV8316Driver<PIN>::getBuckVoltage() {
  uint16_t result = readSPI(Control__6_ADDR);
  Control__6 data;
  data.reg = (result & 0x00FF);
  return (DRV8316_BuckVoltage)data.BUCK_SEL;
};
template <class PIN>
void DRV8316Driver<PIN>::setBuckVoltage(DRV8316_BuckVoltage volts) {
  uint16_t result = readSPI(Control__6_ADDR);
  Control__6 data;
  data.reg = (result & 0x00FF);
  data.BUCK_SEL = volts;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__6_ADDR, data.reg);
};

template <class PIN>
DRV8316_BuckCurrentLimit DRV8316Driver<PIN>::getBuckCurrentLimit() {
  uint16_t result = readSPI(Control__6_ADDR);
  Control__6 data;
  data.reg = (result & 0x00FF);
  return (DRV8316_BuckCurrentLimit)data.BUCK_CL;
};
template <class PIN>
void DRV8316Driver<PIN>::setBuckCurrentLimit(DRV8316_BuckCurrentLimit mamps) {
  uint16_t result = readSPI(Control__6_ADDR);
  Control__6 data;
  data.reg = (result & 0x00FF);
  data.BUCK_CL = mamps;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__6_ADDR, data.reg);
};

template <class PIN>
bool DRV8316Driver<PIN>::isBuckPowerSequencingEnabled() {
  uint16_t result = readSPI(Control__6_ADDR);
  Control__6 data;
  data.reg = (result & 0x00FF);
  return data.BUCK_PS_DIS == BUCK_PS_DIS_ENABLE;
};
template <class PIN>
void DRV8316Driver<PIN>::setBuckPowerSequencingEnabled(bool enabled) {
  uint16_t result = readSPI(Control__6_ADDR);
  Control__6 data;
  data.reg = (result & 0x00FF);
  data.BUCK_PS_DIS = enabled ? BUCK_PS_DIS_ENABLE : BUCK_PS_DIS_DISABLE;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__6_ADDR, data.reg);
};

template <class PIN>
DRV8316_DelayTarget DRV8316Driver<PIN>::getDelayTarget() {
  uint16_t result = readSPI(Control__10_ADDR);
  Control__10 data;
  data.reg = (result & 0x00FF);
  return (DRV8316_DelayTarget)data.DLY_TARGET;
};
template <class PIN>
void DRV8316Driver<PIN>::setDelayTarget(DRV8316_DelayTarget us) {
  uint16_t result = readSPI(Control__10_ADDR);
  Control__10 data;
  data.reg = (result & 0x00FF);
  data.DLY_TARGET = us;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__10_ADDR, data.reg);
};

template <class PIN>
bool DRV8316Driver<PIN>::isDelayCompensationEnabled() {
  uint16_t result = readSPI(Control__10_ADDR);
  Control__10 data;
  data.reg = (result & 0x00FF);
  return data.DLYCMP_EN == DLYCMP_EN_ENABLE;
};
template <class PIN>
void DRV8316Driver<PIN>::setDelayCompensationEnabled(bool enabled) {
  uint16_t result = readSPI(Control__10_ADDR);
  Control__10 data;
  data.reg = (result & 0x00FF);
  data.DLYCMP_EN = enabled ? DLYCMP_EN_ENABLE : DLYCMP_EN_DISABLE;
  delayMicroseconds(1);  // delay at least 400ns between operations
  result = writeSPI(Control__10_ADDR, data.reg);
};

//==============================================================================

inline void DRV8316Status::print() const {
  Serial.print("DRV8316 Status: ");
  Serial.print("FAULT: ");
  Serial.print(isFault());
  Serial.print(", OVP: ");
  Serial.print(isOverVoltage());
  Serial.print(", OCP: ");
  Serial.print(isOverCurrent());
  Serial.print(", OT: ");
  Serial.print(isOverTemperature());
  Serial.print(", SPI_FLT: ");
  Serial.print(isSPIError());
  Serial.print(", BK_FLT: ");
  Serial.print(isBuckError());
  Serial.print(", NPOR: ");
  Serial.println(isPowerOnReset());

  Serial.print("DRV8316 Status1: ");
  Serial.print("OCP_HA: ");
  Serial.print(isOverCurrent_Ah());
  Serial.print(", OCP_LA: ");
  Serial.print(isOverCurrent_Al());
  Serial.print(", OCP_HB: ");
  Serial.print(isOverCurrent_Bh());
  Serial.print(", OCP_LB: ");
  Serial.print(isOverCurrent_Bl());
  Serial.print(", OCP_HC: ");
  Serial.print(isOverCurrent_Ch());
  Serial.print(", OCP_LC: ");
  Serial.println(isOverCurrent_Cl());

  Serial.print("DRV8316 Status2: ");
  Serial.print("OTP_ERR: ");
  Serial.print(isOneTimeProgrammingError());
  Serial.print(", BUCK_OCP: ");
  Serial.print(isBuckOverCurrent());
  Serial.print(", BUCK_UV: ");
  Serial.println(isBuckUnderVoltage());
}
