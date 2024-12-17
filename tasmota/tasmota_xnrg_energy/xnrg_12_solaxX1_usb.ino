/*
  xnrg_12_solaxX1_usb.ino - Solax X1 inverter USB serial support for Tasmota

  Copyright (C) 2024 by Stefan Wershoven

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_ENERGY_SENSOR
#ifdef USE_SOLAX_X1_USB
/*********************************************************************************************\
 * Solax X1 Inverter "USB"
\*********************************************************************************************/

#ifdef USE_SOLAX_X1
#error "USE_SOLAX_X1 and USE_SOLAX_X1_USB cannot be used at the same time!"
#endif

#define XNRG_12            12

#ifndef SOLAXX1_SPEED
#define SOLAXX1_SPEED      9600      // default solax USB speed
#endif

 // look at SetOption65 1 and SetOption36 0

// #define SOLAXX1_READCONFIG          // enable to read inverters config; disable to save codespace (3k1)

#define SOLAXX1_BUFFERSIZE 256

#define D_SOLAX_X1         "SolaxX1"

#include <TasmotaSerial.h>
TasmotaSerial *solaxX1Serial;

const char kSolaxMode[] PROGMEM =
  D_INITIALIZED "|" D_OFF "|" D_SOLAX_MODE_0 "|" D_SOLAX_MODE_1 "|" D_SOLAX_MODE_2 "|" D_SOLAX_MODE_3 "|"  D_SOLAX_MODE_4 "|"
  D_SOLAX_MODE_5 "|" D_SOLAX_MODE_6;

const char kSolaxError[] PROGMEM =
  D_SOLAX_ERROR_0 "|" D_SOLAX_ERROR_1 "|" D_SOLAX_ERROR_2 "|" D_SOLAX_ERROR_3 "|" D_SOLAX_ERROR_4 "|" D_SOLAX_ERROR_5 "|"
  D_SOLAX_ERROR_6 "|" D_SOLAX_ERROR_7 "|" D_SOLAX_ERROR_8;

#ifdef SOLAXX1_READCONFIG
const char kSolaxSafetyType[] PROGMEM =
  "VDE 0126|VDE-AR-N 4105|AS 4777|G98|C10/11|ÖVE/ÖNORM E 8001|EN 50438 NL|EN 50438 DK|CEB|CEI021|NRS 097-2-1|VDE 0126 Greece/Iceland|"
  "UTE C15-712|IEC 61727|G99|VDE 0126 Greece/Co|Guyana|C15-712 France/Iceland 50|C15-712 France/Iceland 60|New Zeeland|RD1699|Chile|"
  "EN 50438 Ireland|Philippines|Czech PPDS|Czech 50438|EN 50549 EU|Denmark 2019 EU|RD 1699 Island|EN50549 Poland|MEA Thailand|"
  "PEA Thailand|ACEA|AS 4777 2020 B|AS 4777 2020 C|Sri Lanka|BRAZIL 240|EN 50549 SK|EN 50549 EU|G98/NI|Denmark 2019 EU|RD 1699 Island";
#endif // SOLAXX1_READCONFIG

union {
  uint32_t ErrMessage;
  struct {
    //BYTE0
    uint8_t TzProtectFault:1;//0
    uint8_t MainsLostFault:1;//1
    uint8_t GridVoltFault:1;//2
    uint8_t GridFreqFault:1;//3
    uint8_t PLLLostFault:1;//4
    uint8_t BusVoltFault:1;//5
    uint8_t ErrBit06:1;//6
    uint8_t OciFault:1;//7
    //BYTE1
    uint8_t Dci_OCP_Fault:1;//8
    uint8_t ResidualCurrentFault:1;//9
    uint8_t PvVoltFault:1;//10
    uint8_t Ac10Mins_Voltage_Fault:1;//11
    uint8_t IsolationFault:1;//12
    uint8_t TemperatureOverFault:1;//13
    uint8_t FanFault:1;//14
    uint8_t ErrBit15:1;//15
    //BYTE2
    uint8_t SpiCommsFault:1;//16
    uint8_t SciCommsFault:1;//17
    uint8_t ErrBit18:1;//18
    uint8_t InputConfigFault:1;//19
    uint8_t EepromFault:1;//20
    uint8_t RelayFault:1;//21
    uint8_t SampleConsistenceFault:1;//22
    uint8_t ResidualCurrent_DeviceFault:1;//23
    //BYTE3
    uint8_t ErrBit24:1;//24
    uint8_t ErrBit25:1;//25
    uint8_t ErrBit26:1;//26
    uint8_t ErrBit27:1;//27
    uint8_t ErrBit28:1;//28
    uint8_t DCI_DeviceFault:1;//29
    uint8_t OtherDeviceFault:1;//30
    uint8_t ErrBit31:1;//31
  };
} solaxX1_ErrCode;

struct SOLAXX1_LIVEDATA {
  int16_t temperature = 0;
  float energy_today = 0;
  float energy_total = 0;
  float dc1_voltage = 0;
  float dc2_voltage = 0;
  float dc1_current = 0;
  float dc2_current = 0;
  uint32_t runtime_total = 0;
  float dc1_power = 0;
  float dc2_power = 0;
  int16_t runMode = -2;
  uint32_t errorCode = 0;
  uint8_t SerialNumber[16] = {0x00};
  uint16_t ModelCode = 0;
  uint16_t ModelType = 0;
} solaxX1;

struct SOLAXX1_GLOBALDATA {
  bool SolaxUSBregistered = false;
  uint8_t SendRetry_count = 5;
  uint8_t QueryData_count = 0;

  bool Command_QueryID = false;
  bool Command_QueryConfig = false;
  bool MeterMode = false;
  float MeterPower = 5000;
  float MeterImport;
  float MeterExport;
} solaxX1_global;

/*********************************************************************************************/

void solaxX1_RegisterSolaxUSB(void) {
//  uint8_t RegisterUSBMsg[17] = {0xaa, 0x55, 0x11, 0x02, 0x01, 0x53, 0x57, 0x41, 0x4D, 0x54, 0x4C, 0x59, 0x34, 0x5A, 0x4D, 0x1f, 0x04};
  uint8_t RegisterUSBMsg[12] = {0xaa, 0x55, 0x0E, 0x02, 0x01, 0x54, 0x41, 0x53, 0x4D, 0x4F, 0x54, 0x41};
  solaxX1_USBSendRaw(RegisterUSBMsg, 12);
}

void solaxX1_RequestLiveDataUSB(void) {
//  uint8_t RequestLiveDataMsg[7] = {0xaa, 0x55, 0x07, 0x01, 0x0C, 0x13, 0x01};
  uint8_t RequestLiveDataMsg[5] = {0xaa, 0x55, 0x07, 0x01, 0x0C};
  solaxX1_USBSendRaw(RequestLiveDataMsg, 5);
}

void solaxX1_RequestSerialNumberUSB(void) {
//  uint8_t RequestSerialNumberMsg[7] = {0xaa, 0x55, 0x07, 0x01, 0x05, 0x0c, 0x01};
  uint8_t RequestSerialNumberMsg[5] = {0xaa, 0x55, 0x07, 0x01, 0x05};
  solaxX1_USBSendRaw(RequestSerialNumberMsg, 5);
}

void solaxX1_RequestConfigUSB(void) {
  uint8_t RequestConfigMsg[5] = {0xaa, 0x55, 0x07, 0x01, 0x16};
  solaxX1_USBSendRaw(RequestConfigMsg, 5);
}

void solaxX1_USBSendRaw(uint8_t *SendBuffer, uint8_t DataLen) {
  uint16_t crc;
  while (solaxX1Serial->available()) { // read serial if any old data is available
    solaxX1Serial->read();
  }
  solaxX1Serial->flush();
  solaxX1Serial->write(SendBuffer, DataLen);
  crc = solaxX1_calculateCRC(SendBuffer, DataLen); // Use CRC Solax algorithm
  solaxX1Serial->write(lowByte(crc));
  solaxX1Serial->write(highByte(crc));
  solaxX1Serial->flush();
  AddLogBuffer(LOG_LEVEL_DEBUG_MORE, SendBuffer, DataLen);
}

bool solaxX1_USBReceive(uint8_t *ReadBuffer) {
  uint8_t SerAvial;
  uint8_t len = 0;

  while (SerAvial = solaxX1Serial->available()) {
    while (SerAvial--) {
      ReadBuffer[len++] = (uint8_t)solaxX1Serial->read();
    }
    delay(10); // wait for more data, because of slowness of the inverter
  }
  AddLogBuffer(LOG_LEVEL_DEBUG_MORE, ReadBuffer, len);

  // received "USB Response register pocket dongle serial number" -> ignore checksum as it is not present
  if (ReadBuffer[3] == 0x02 && ReadBuffer[4] == 0x01) return false;

  uint16_t crc = solaxX1_calculateCRC(ReadBuffer, len - 2); // calculate out crc bytes
  return !(ReadBuffer[len - 2] == lowByte(crc) && ReadBuffer[len - 1] == highByte(crc));
}

uint16_t solaxX1_calculateCRC(uint8_t *bExternTxPackage, uint8_t bLen) {
  uint8_t i;
  uint16_t wChkSum = 0;
  for (i = 0; i < bLen; i++) {
    wChkSum += bExternTxPackage[i];
  }
  return wChkSum;
}

void solaxX1_ExtractText(uint8_t *DataIn, uint8_t *DataOut, uint8_t Begin, uint8_t End) {
  uint8_t i;
  for (i = Begin; i <= End; i++) {
    DataOut[i - Begin] = DataIn[i];
  }
  DataOut[End - Begin + 1] = 0;
}

uint8_t solaxX1_ParseErrorCode(uint32_t code) {
  solaxX1_ErrCode.ErrMessage = code;
  if (code == 0) return 0;
  if (solaxX1_ErrCode.MainsLostFault) return 1;
  if (solaxX1_ErrCode.GridVoltFault) return 2;
  if (solaxX1_ErrCode.GridFreqFault) return 3;
  if (solaxX1_ErrCode.PvVoltFault) return 4;
  if (solaxX1_ErrCode.IsolationFault) return 5;
  if (solaxX1_ErrCode.TemperatureOverFault) return 6;
  if (solaxX1_ErrCode.FanFault) return 7;
  if (solaxX1_ErrCode.OtherDeviceFault) return 8;
  return 0;
}

void solaxX1_SwitchMeterMode(bool MeterMode) {
  if (solaxX1_global.MeterMode == MeterMode) return;
  solaxX1_global.MeterMode = MeterMode;
  if (MeterMode) {
    Energy->data_valid[0] = ENERGY_WATCHDOG;
    solaxX1.runMode = -2;
    solaxX1.temperature = solaxX1.dc1_voltage =  solaxX1.dc1_current = solaxX1.dc1_power = solaxX1.dc2_voltage = solaxX1.dc2_current = solaxX1.dc2_power = 0;
  } else {
    solaxX1.runMode = -1;
  }
}

/*********************************************************************************************/

void solaxX1_CyclicTask(void) { // Every 100/250 milliseconds
  uint8_t DataRead[SOLAXX1_BUFFERSIZE] = {0};
  uint8_t TempData[16] = {0};
  char TempDataChar[32];
  float TempFloat;
  static uint32_t LastMeterTime;

  if (solaxX1Serial->available()) {
    if (solaxX1_USBReceive(DataRead)) { // CRC or other error -> no further action
      AddLog(LOG_LEVEL_ERROR, PSTR("SX1: CRC error in received data"));
      return;
    }
  
    if (DataRead[0] != 0xAA || DataRead[1] != 0x55) { // Check for header
      AddLog(LOG_LEVEL_ERROR, PSTR("SX1: Header check failed: %02X %02X"), DataRead[0], DataRead[1]);
      return;
    }

    solaxX1_global.SendRetry_count = 20; // Inverter is responding

    if (DataRead[3] == 0x01 && DataRead[4] == 0x8C) { // received response RequestLiveDataUSB
      Energy->data_valid[0] = 0;
      solaxX1.temperature =       (DataRead[84] << 8)  |  DataRead[83]; // Temperature
      solaxX1.energy_today =     ((DataRead[32] << 8)  |  DataRead[31]) * 0.1f; // Energy Today
//AddLog(LOG_LEVEL_INFO, PSTR("SX1: today %1_f"), &solaxX1.energy_today);
//RtcSettings.energy_kWhtoday_ph[i]) / 100000
//      Energy->daily[0] =         ((DataRead[32] << 8)  |  DataRead[31]) * 0.1f; // Energy Today
//      Energy->kWhtoday[0] =  ((DataRead[32] << 8)  |  DataRead[31]) * 10000; // Energy Today
      solaxX1.dc1_voltage =      ((DataRead[12] << 8)  |  DataRead[11]) * 0.1f; // PV1 Voltage
      solaxX1.dc2_voltage =      ((DataRead[14] << 8)  |  DataRead[13]) * 0.1f; // PV2 Voltage
      solaxX1.dc1_current =      ((DataRead[16] << 8)  |  DataRead[15]) * 0.1f; // PV1 Current
      solaxX1.dc2_current =      ((DataRead[18] << 8)  |  DataRead[17]) * 0.1f; // PV2 Current
      Energy->current[0] =       ((DataRead[8] << 8)   |  DataRead[7])  * 0.1f; // AC Current
      Energy->voltage[0] =       ((DataRead[6] << 8)   |  DataRead[5])  * 0.1f; // AC Voltage
      Energy->frequency[0] =     ((DataRead[24] << 8)  |  DataRead[23]) * 0.01f; // AC Frequency
      Energy->active_power[0] =  ((DataRead[10] << 8)  |  DataRead[9]); // AC Power
      solaxX1.energy_total =     ((DataRead[30] << 24) | (DataRead[29] << 16) | (DataRead[28] << 8) | DataRead[27]) * 0.1f; // Energy Total
      uint32_t runtime_total =    (DataRead[90] << 24) | (DataRead[89] << 16) | (DataRead[88] << 8) | DataRead[87]; // Work Time Total
      if (runtime_total) solaxX1.runtime_total = runtime_total; // Work Time valid
      solaxX1.runMode =           (DataRead[26] << 8)  |  DataRead[25]; // Work mode
//      solaxX1.errorCode =         (DataRead[58] << 24) | (DataRead[57] << 16) | (DataRead[56] << 8) | DataRead[55]; // Error Code
      solaxX1.dc1_power = solaxX1.dc1_voltage * solaxX1.dc1_current;
      solaxX1.dc2_power = solaxX1.dc2_voltage * solaxX1.dc2_current;
      if (Settings->flag3.hardware_energy_total) {   // SetOption72 - Enable hardware energy total counter as reference (#6561)
        Energy->import_active[0] = solaxX1.energy_total;
        EnergyUpdateTotal();  // 484.708 kWh
        EnergySaveState();
      }
      DEBUG_SENSOR_LOG(PSTR("SX1: received response RequestLiveDataUSB"));
      return;
    } // end received response RequestLiveDataUSB"

    if (DataRead[3] == 0x01 && DataRead[4] == 0x85) { // received response RequestSerialNumberUSB
      solaxX1_ExtractText(DataRead, solaxX1.SerialNumber, 5, 18); // extract inverter serial number
      solaxX1.ModelCode = (DataRead[34] << 8) | DataRead[33];
      solaxX1.ModelType = (DataRead[44] << 8) | DataRead[43];
      DEBUG_SENSOR_LOG(PSTR("SX1: received response RequestSerialNumberUSB"));
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: Inverter SN: %s, ModelCode: 0x%04X, ModelType: 0x%04X"), (char*)solaxX1.SerialNumber, solaxX1.ModelCode, solaxX1.ModelType);
      return;
    } // end received response solaxX1_RequestSerialNumberUSB

    if (DataRead[3] == 0x02 && DataRead[4] == 0x01) { // received response RegisterSolaxUSB
      solaxX1_global.QueryData_count = 5; // give time for next query
      solaxX1_global.SolaxUSBregistered = true;
      DEBUG_SENSOR_LOG(PSTR("SX1: received response RegisterSolaxUSB"));
      return;
    } // end received response RegisterSolaxUSB

#ifdef SOLAXX1_READCONFIG
    if (DataRead[3] == 0x01 && DataRead[4] == 0x96) { // received "Response for query (config data)"
      // This values are displayed as they were received from the inverter. They are not interpreted in any way.
      TempFloat = ((DataRead[9] << 8) | DataRead[10]) * 0.1f;
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wVpvStart: %1_f V (Inverter launch voltage threshold)"), &TempFloat);
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wTimeStart: %d sec (launch wait time)"), (DataRead[285] << 8) | DataRead[286]);
      TempFloat = ((DataRead[13] << 8) | DataRead[14]) * 0.1f;
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wVacMinProtect: %1_f V (allowed minimum grid voltage)"), &TempFloat);
      TempFloat = ((DataRead[15] << 8) | DataRead[16]) * 0.1f;
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wVacMaxProtect: %1_f V (allowed maximum grid voltage)"), &TempFloat);
      TempFloat = ((DataRead[17] << 8) | DataRead[18]) * 0.01f;
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wFacMinProtect: %2_f Hz (allowed minimum grid frequency)"), &TempFloat);
      TempFloat = ((DataRead[19] << 8) | DataRead[20]) * 0.01f;
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wFacMaxProtect: %2_f Hz (allowed maximum grid frequency)"), &TempFloat);
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wDciLimits: %d mA (DC component limits)"), (DataRead[21] << 8) | DataRead[22]);
      TempFloat = ((DataRead[23] << 8) | DataRead[24]) * 0.1f;
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wGrid10MinAvgProtect: %1_f V (10 minutes over voltage protect)"), &TempFloat);
      TempFloat = ((DataRead[25] << 8) | DataRead[26]) * 0.1f;
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wVacMinSlowProtect: %1_f V (grid undervoltage protect value)"), &TempFloat);
      TempFloat = ((DataRead[27] << 8) | DataRead[28]) * 0.1f;
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wVacMaxSlowProtect: %1_f V (grid overvoltage protect value)"), &TempFloat);
      TempFloat = ((DataRead[29] << 8) | DataRead[30]) * 0.01f;
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wFacMinSlowProtect: %2_f Hz (grid underfrequency protect value)"), &TempFloat);
      TempFloat = ((DataRead[31] << 8) | DataRead[32]) * 0.01f;
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wFacMaxSlowProtect: %2_f Hz (grid overfrequency protect value)"), &TempFloat);
      GetTextIndexed(TempDataChar, sizeof(TempDataChar), (DataRead[33] << 8) | DataRead[34], kSolaxSafetyType);
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wSafety: %d ≙ %s"), (DataRead[33] << 8) | DataRead[34], TempDataChar);
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wPowerfactor_mode: %d"), DataRead[35]);
      TempFloat = DataRead[36] * 0.01f;
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wPowerfactor_data: %2_f"), &TempFloat);
      TempFloat = DataRead[37] * 0.01f;
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wUpperLimit: %2_f (overexcite limits)"), &TempFloat);
      TempFloat = DataRead[38] * 0.01f;
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wLowerLimit: %2_f (underexcite limits)"), &TempFloat);
      TempFloat = DataRead[39] * 0.01f;
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wPowerLow: %2_f (power ratio change upper limits)"), &TempFloat);
      TempFloat = DataRead[40] * 0.01f;
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wPowerUp: %2_f (power ratio change lower limits)"), &TempFloat);
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: Qpower_set: %d"), (DataRead[41] << 8) | DataRead[42]);
      TempFloat = ((DataRead[43] << 8) | DataRead[44]) * 0.01f;
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: WFreqSetPoint: %2_f Hz (Over Frequency drop output setpoint)"), &TempFloat);
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: WFreqDroopRate: %d %% (drop output slope)"), (DataRead[45] << 8) | DataRead[46]);
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: QuVupRate: %d %% (Q(U) curve up set point)"), (DataRead[47] << 8) | DataRead[48]);
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: QuVlowRate: %d %% (Q(U) curve low set point)"), (DataRead[49] << 8) | DataRead[50]);
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: WPowerLimitsPercent: %d %%"), (DataRead[51] << 8) | DataRead[52]);
      TempFloat = ((DataRead[53] << 8) | DataRead[54]) * 0.01f;
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: WWgra: %2_f %%"), &TempFloat);
      TempFloat = ((DataRead[55] << 8) | DataRead[56]) * 0.1f;
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wWv2: %1_f V"), &TempFloat);
      TempFloat = ((DataRead[57] << 8) | DataRead[58]) * 0.1f;
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wWv3: %1_f V"), &TempFloat);
      TempFloat = ((DataRead[59] << 8) | DataRead[60]) * 0.1f;
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wWv4: %1_f V"), &TempFloat);
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wQurangeV1: %d %%"), (DataRead[61] << 8) | DataRead[62]);
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: wQurangeV4: %d %%"), (DataRead[63] << 8) | DataRead[64]);
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: BVoltPowerLimit: %d"), (DataRead[65] << 8) | DataRead[66]);
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: WPowerManagerEnable: %d"), (DataRead[67] << 8) | DataRead[68]);
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: WGlobalSeachMPPTStrartFlg: %d"), (DataRead[69] << 8) | DataRead[70]);
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: WFrqProtectRestrictive: %d"), (DataRead[71] << 8) | DataRead[72]);
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: WQuDelayTimer: %d sec"), (DataRead[73] << 8) | DataRead[74]);
      AddLog(LOG_LEVEL_INFO, PSTR("SX1: WFreqActivePowerDelayTimer: %d ms"), (DataRead[75] << 8) | DataRead[76]);
      return;
    } // end received "Response for query (config data)"
#endif // SOLAXX1_READCONFIG

  } // end solaxX1Serial->available()

//  DEBUG_SENSOR_LOG(PSTR("SX1: solaxX1_global.QueryData_count: %d, solaxX1_global.SendRetry_count: %d"), solaxX1_global.QueryData_count, solaxX1_global.SendRetry_count);
  if (solaxX1_global.SolaxUSBregistered) {
    if (!solaxX1_global.QueryData_count) { // normal periodically query
      solaxX1_global.QueryData_count = 3;
      if (!solaxX1.SerialNumber[0] || solaxX1_global.Command_QueryID) { // Query serial number
        solaxX1_global.Command_QueryID = false;
        DEBUG_SENSOR_LOG(PSTR("SX1: Send RequestSerialNumberUSB"));
        solaxX1_RequestSerialNumberUSB();
      } else if (solaxX1_global.Command_QueryConfig) { // Config query
        solaxX1_global.Command_QueryConfig = false;
        DEBUG_SENSOR_LOG(PSTR("SX1: Send config query"));
        solaxX1_RequestConfigUSB();
      } else { // live query
        DEBUG_SENSOR_LOG(PSTR("SX1: Send RequestLiveDataUSB"));
        solaxX1_RequestLiveDataUSB();
      }
    }  // end normal periodically query
    solaxX1_global.QueryData_count--;
    if (!solaxX1_global.SendRetry_count) { // Inverter went "off"
      solaxX1_global.SendRetry_count = 20;
      DEBUG_SENSOR_LOG(PSTR("SX1: Inverter went \"off\""));
      Energy->data_valid[0] = ENERGY_WATCHDOG;
      solaxX1.temperature = solaxX1.dc1_voltage =  solaxX1.dc1_current = solaxX1.dc1_power = solaxX1.dc2_voltage = solaxX1.dc2_current = solaxX1.dc2_power = 0;
      solaxX1.runMode = -1; // off(line)
      solaxX1_global.SolaxUSBregistered = false;
    } // end Inverter went "off"
  } else { // sent query for inverters in offline status
    if (!solaxX1_global.SendRetry_count) {
      solaxX1_global.Command_QueryConfig = solaxX1_global.Command_QueryID = false; // Clear commands to be sure
      solaxX1_global.SendRetry_count = 20;
      DEBUG_SENSOR_LOG(PSTR("SX1: Sent RegisterSolaxUSB"));
      solaxX1_RegisterSolaxUSB();
    }
  }
  solaxX1_global.SendRetry_count--;
return;  
} // end solaxX1_CyclicTask

void solaxX1_EverySecond(void) {
//  AddLog(LOG_LEVEL_DEBUG, PSTR("SX1: Energy->data_valid[0]: %d"), Energy->data_valid[0]);
  if (Settings->flag3.hardware_energy_total) return;   // SetOption72 - Enable hardware energy total counter as reference (#6561)
  if (Energy->data_valid[0]) return;

  Energy->kWhtoday_delta[0] += Energy->active_power[0] * 1000 / 36;
  EnergyUpdateToday();

//  int32_t tdiff = abs(Settings->energy_kWhtoday_ph[0] - RtcSettings.energy_kWhtoday_ph[0]);
//  AddLog(LOG_LEVEL_DEBUG, PSTR("SX1: imp_active: %1_f, set: %d, rtc: %d, tdiff: %d"), &Energy->import_active[0], Settings->energy_kWhtoday_ph[0], RtcSettings.energy_kWhtoday_ph[0], tdiff);
//  AddLog(LOG_LEVEL_DEBUG, PSTR("SX1: moduptime: %d, tdiff: %d"), TasmotaGlobal.uptime % 300, RtcSettings.energy_kWhtoday_ph[0] - Settings->energy_kWhtoday_ph[0]);
//  if (tdiff > 1000) EnergySaveState();
  if ((RtcSettings.energy_kWhtoday_ph[0] - Settings->energy_kWhtoday_ph[0]) && !(TasmotaGlobal.uptime % 300)) EnergySaveState();
} // end solaxX1_EverySecond

void solaxX1_SnsInit(void) {
  AddLog(LOG_LEVEL_INFO, PSTR("SX1: Init - RX-pin: %d, TX-pin: %d"), Pin(GPIO_SOLAXX1_RX), Pin(GPIO_SOLAXX1_TX));
  solaxX1Serial = new TasmotaSerial(Pin(GPIO_SOLAXX1_RX), Pin(GPIO_SOLAXX1_TX), 1, 1, SOLAXX1_BUFFERSIZE);
  if (solaxX1Serial->begin(SOLAXX1_SPEED)) {
    if (solaxX1Serial->hardwareSerial()) ClaimSerial();
#ifdef ESP32
    AddLog(LOG_LEVEL_DEBUG, PSTR("SX1: Serial UART%d"), solaxX1Serial->getUart());
#endif
  } else {
    TasmotaGlobal.energy_driver = ENERGY_NONE;
  }
}

void solaxX1_DrvInit(void) {
  if (PinUsed(GPIO_SOLAXX1_RX) && PinUsed(GPIO_SOLAXX1_TX)) {
    TasmotaGlobal.energy_driver = XNRG_12;
    Energy->type_dc = true; // Handle like DC, because U*I from inverter is not valid for apparent power; U*I could be lower than active power
    Energy->frequency[0] = 0; // Set value, to make frequency present in output
  }
}

bool SolaxX1_cmd(void) {
  if (Energy->command_code != CMND_ENERGYCONFIG) return false; // Process unchanged data

   if (!strncasecmp(XdrvMailbox.data, "MeterPower", 10)) {
    solaxX1_global.MeterPower = CharToFloat(&XdrvMailbox.data[11]);
    ResponseCmndFloat(solaxX1_global.MeterPower, 1);
    AddLog(LOG_LEVEL_DEBUG, PSTR("SX1: MeterPower: %3_f"), &solaxX1_global.MeterPower);
    return false;
   }

  if (!solaxX1_global.SolaxUSBregistered) {
    AddLog(LOG_LEVEL_INFO, PSTR("SX1: No inverter registered"));
    return false;
  }

  if (!strcasecmp(XdrvMailbox.data, "ReadIDinfo")) {
    solaxX1_global.Command_QueryID = true;
    return true;
  } else if (!strcasecmp(XdrvMailbox.data, "ReadConfig")) {
#ifdef SOLAXX1_READCONFIG
    solaxX1_global.Command_QueryConfig = true;
    return true;
#else
    AddLog(LOG_LEVEL_INFO, PSTR("SX1: Command not available. Please set compiler directive '#define SOLAXX1_READCONFIG'."));
    return false;
#endif // SOLAXX1_READCONFIG
  }
  AddLog(LOG_LEVEL_INFO, PSTR("SX1: Unknown command: \"%s\""),XdrvMailbox.data);
  return false;
}

#ifdef USE_WEBSERVER
const char HTTP_SNS_solaxX1_Num[] PROGMEM = "{s}" D_SOLAX_X1 " %s{m}</td><td style='text-align:%s'>%s{m}{m}%s{e}";
const char HTTP_SNS_solaxX1_Str[] PROGMEM = "{s}" D_SOLAX_X1 " %s</td><td style='text-align:right'>%s{e}";
const char HTTP_SNS_solaxX1_Mtr[] PROGMEM = "{s}" D_GATEWAY " %s{m}</td><td style='text-align:%s'>%s{m}{m}%s{e}";
#endif // USE_WEBSERVER

void solaxX1_Show(uint32_t function) {
  char solar_power[33];
  dtostrfd(solaxX1.dc1_power + solaxX1.dc2_power, Settings->flag2.wattage_resolution, solar_power);
  char pv1_voltage[33];
  dtostrfd(solaxX1.dc1_voltage, Settings->flag2.voltage_resolution, pv1_voltage);
  char pv1_current[33];
  dtostrfd(solaxX1.dc1_current, Settings->flag2.current_resolution, pv1_current);
  char pv1_power[33];
  dtostrfd(solaxX1.dc1_power, Settings->flag2.wattage_resolution, pv1_power);
#ifdef SOLAXX1_PV2
  char pv2_voltage[33];
  dtostrfd(solaxX1.dc2_voltage, Settings->flag2.voltage_resolution, pv2_voltage);
  char pv2_current[33];
  dtostrfd(solaxX1.dc2_current, Settings->flag2.current_resolution, pv2_current);
  char pv2_power[33];
  dtostrfd(solaxX1.dc2_power, Settings->flag2.wattage_resolution, pv2_power);
#endif
  char inverter_today[33];
  dtostrfd(solaxX1.energy_today, Settings->flag2.energy_resolution, inverter_today);
  char inverter_total[33];
  dtostrfd(solaxX1.energy_total, Settings->flag2.energy_resolution, inverter_total);
  char status[33];
  GetTextIndexed(status, sizeof(status), solaxX1.runMode + 2, kSolaxMode);

  switch (function) {
    case FUNC_JSON_APPEND:
      ResponseAppend_P(PSTR(",\"" D_JSON_SOLAR_POWER "\":%s,\"" D_JSON_PV1_VOLTAGE "\":%s,\"" D_JSON_PV1_CURRENT "\":%s,\"" D_JSON_PV1_POWER "\":%s"),
                                  solar_power, pv1_voltage, pv1_current, pv1_power);
#ifdef SOLAXX1_PV2
      ResponseAppend_P(PSTR(",\"" D_JSON_PV2_VOLTAGE "\":%s,\"" D_JSON_PV2_CURRENT "\":%s,\"" D_JSON_PV2_POWER "\":%s"),
                                  pv2_voltage, pv2_current, pv2_power);
#endif
      ResponseAppend_P(PSTR(",\"" D_JSON_TEMPERATURE "\":%d,\"" D_JSON_RUNTIME "\":%d,\"" D_JSON_STATUS "\":\"%s\",\"" D_JSON_ERROR "\":%d"),
                                  solaxX1.temperature, solaxX1.runtime_total, status, solaxX1.errorCode);

#ifdef USE_DOMOTICZ
      // Avoid bad temperature report at beginning of the day (spikes of 1200 celsius degrees)
      if (0 == TasmotaGlobal.tele_period && solaxX1.temperature < 100) { DomoticzSensor(DZ_TEMP, solaxX1.temperature); }
#endif // USE_DOMOTICZ
      break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_COL_SENSOR: {
      String table_align = Settings->flag5.gui_table_align?"right":"left";
      if (solaxX1_global.MeterMode) {
        char TempDataChar[33];
        WSContentSend_P(PSTR("<tr><td colspan=5 style='font-size:2px'><hr size=1/>{e}"));
        dtostrfd(solaxX1_global.MeterPower, Settings->flag2.wattage_resolution, TempDataChar);
        WSContentSend_PD(HTTP_SNS_solaxX1_Mtr, D_POWERUSAGE, table_align.c_str(), TempDataChar, D_UNIT_WATT);
        dtostrfd(solaxX1_global.MeterImport, Settings->flag2.energy_resolution, TempDataChar);
        WSContentSend_PD(HTTP_SNS_solaxX1_Mtr, "Import", table_align.c_str(), TempDataChar, D_UNIT_KILOWATTHOUR);
        dtostrfd(solaxX1_global.MeterExport, Settings->flag2.energy_resolution, TempDataChar);
        WSContentSend_PD(HTTP_SNS_solaxX1_Mtr, "Export", table_align.c_str(), TempDataChar, D_UNIT_KILOWATTHOUR);
        return;
      }
      static uint32_t LastOnlineTime;
      if (solaxX1.runMode != -1) LastOnlineTime = TasmotaGlobal.uptime;
      if (TasmotaGlobal.uptime < LastOnlineTime + 300) { // Hide numeric live data, when inverter is offline for more than 5 min
#ifdef SOLAXX1_PV2
        WSContentSend_PD(HTTP_SNS_solaxX1_Num, D_SOLAR_POWER, table_align.c_str(), solar_power, D_UNIT_WATT);
#endif
        WSContentSend_PD(HTTP_SNS_solaxX1_Num, D_PV1_VOLTAGE, table_align.c_str(), pv1_voltage, D_UNIT_VOLT);
        WSContentSend_PD(HTTP_SNS_solaxX1_Num, D_PV1_CURRENT, table_align.c_str(), pv1_current, D_UNIT_AMPERE);
        WSContentSend_PD(HTTP_SNS_solaxX1_Num, D_PV1_POWER,   table_align.c_str(), pv1_power,   D_UNIT_WATT);
#ifdef SOLAXX1_PV2
        WSContentSend_PD(HTTP_SNS_solaxX1_Num, D_PV2_VOLTAGE, table_align.c_str(), pv2_voltage, D_UNIT_VOLT);
        WSContentSend_PD(HTTP_SNS_solaxX1_Num, D_PV2_CURRENT, table_align.c_str(), pv2_current, D_UNIT_AMPERE);
        WSContentSend_PD(HTTP_SNS_solaxX1_Num, D_PV2_POWER,   table_align.c_str(), pv2_power,   D_UNIT_WATT);
#endif
        if (!Settings->flag3.hardware_energy_total) {   // SetOption72 - Enable hardware energy total counter as reference (#6561)
          WSContentSend_PD(HTTP_SNS_solaxX1_Num, D_ENERGY_TODAY, table_align.c_str(), inverter_today, D_UNIT_KILOWATTHOUR);
          WSContentSend_PD(HTTP_SNS_solaxX1_Num, D_ENERGY_TOTAL, table_align.c_str(), inverter_total, D_UNIT_KILOWATTHOUR);
        }
        char SXTemperature[16];
        dtostrfd(solaxX1.temperature, Settings->flag2.temperature_resolution, SXTemperature);
        WSContentSend_PD(HTTP_SNS_solaxX1_Num, D_TEMPERATURE, table_align.c_str(), SXTemperature, D_UNIT_DEGREE D_UNIT_CELSIUS);
      }
      WSContentSend_P(HTTP_SNS_solaxX1_Num, D_UPTIME, table_align.c_str(), String(solaxX1.runtime_total).c_str(), D_UNIT_HOUR);
      break; }
    case FUNC_WEB_SENSOR:
      if (solaxX1_global.MeterMode) return;
      char errorCodeString[33];
      WSContentSend_P(HTTP_SNS_solaxX1_Str, D_STATUS, status);
//      WSContentSend_P(HTTP_SNS_solaxX1_Str, D_ERROR, GetTextIndexed(errorCodeString, sizeof(errorCodeString), solaxX1_ParseErrorCode(solaxX1.errorCode), kSolaxError));
      if (solaxX1.SerialNumber[0]) WSContentSend_P(HTTP_SNS_solaxX1_Str, "Inverter SN", solaxX1.SerialNumber);
      break;
#endif  // USE_WEBSERVER
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xnrg12(uint32_t function) {
  bool result = false;

  switch (function) {
    case FUNC_EVERY_100_MSECOND:
      if (solaxX1_global.MeterMode) solaxX1_CyclicTask();
      break;
    case FUNC_EVERY_250_MSECOND:
      if (!solaxX1_global.MeterMode) solaxX1_CyclicTask();
      break;
    case FUNC_EVERY_SECOND:
      solaxX1_EverySecond();
      break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_COL_SENSOR:
    case FUNC_WEB_SENSOR:
#endif  // USE_WEBSERVER
    case FUNC_JSON_APPEND:
      solaxX1_Show(function);
      break;
    case FUNC_INIT:
      solaxX1_SnsInit();
      break;
    case FUNC_PRE_INIT:
      solaxX1_DrvInit();
      break;
    case FUNC_COMMAND:
      result = SolaxX1_cmd();
      break;
  }
  return result;
}

#endif  // USE_SOLAX_X1_USB
#endif  // USE_ENERGY_SENSOR
