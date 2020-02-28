/*
 * Copyright (c) 2020 Karsten Becker All rights reserved.
 * Use of this source code is governed by a BSD-style
 * license that can be found in the LICENSE file.
 */
#include <Preferences.h>
#include <SHIBME680.h>
#include <SHIHardware.h>
#include <SHISensor.h>
#include <bsec.h>

const char *SENSOR_STATE = "sensorState";
struct BSEC_State_t {
  uint32_t canary;
  uint8_t serialized_state[BSEC_MAX_STATE_BLOB_SIZE];
  uint32_t n_serialized_state_max = BSEC_MAX_STATE_BLOB_SIZE;
  uint32_t n_serialized_state = BSEC_MAX_STATE_BLOB_SIZE;
  uint8_t work_buffer_state[BSEC_MAX_STATE_BLOB_SIZE];
  uint32_t n_work_buffer_size = BSEC_MAX_STATE_BLOB_SIZE;
};

void SHI::BME680::storeState() {
  SHI_LOGINFO("Storing state");
  BSEC_State_t state;
  bsec_get_state(0, state.serialized_state, state.n_serialized_state_max,
                 state.work_buffer_state, state.n_work_buffer_size,
                 &state.n_serialized_state);
  state.canary = 0xCAFEBABE;
  bsecStatePrefs.putBytes(SENSOR_STATE, &state, sizeof(BSEC_State_t));
}

bool SHI::BME680::restoreState() {
  BSEC_State_t state;
  bsecStatePrefs.getBytes(SENSOR_STATE, &state, sizeof(BSEC_State_t));
  // Algorithm state is stored in 'serialized_state'
  if (state.canary == 0xCAFEBABE) {
    SHI_LOGINFO("Restoring State");
    bsec_set_state(state.serialized_state, state.n_serialized_state,
                   state.work_buffer_state, state.n_work_buffer_size);
    return true;
  }
  return false;
}

bool SHI::BME680::setupSensor() {
  bsecStatePrefs.begin(SENSOR_STATE);
  // We need to use Wire1 as Wire is used by the display
  if (!wire->begin()) {
    SHI_LOGINFO("Failed to init I2C!");
    return false;
  }
  iaqSensor.begin(
      primaryAddress ? BME680_I2C_ADDR_PRIMARY : BME680_I2C_ADDR_SECONDARY,
      *wire);
  SHI_LOGINFO(("BSEC library version " + String(iaqSensor.version.major) + "." +
               String(iaqSensor.version.minor) + "." +
               String(iaqSensor.version.major_bugfix) + "." +
               String(iaqSensor.version.minor_bugfix))
                  .c_str());

  updateIaqSensorStatus();

  restoreState();

  bsec_virtual_sensor_t sensorList[10] = {
      BSEC_OUTPUT_RAW_TEMPERATURE,
      BSEC_OUTPUT_RAW_PRESSURE,
      BSEC_OUTPUT_RAW_HUMIDITY,
      BSEC_OUTPUT_RAW_GAS,
      BSEC_OUTPUT_IAQ,
      BSEC_OUTPUT_STATIC_IAQ,
      BSEC_OUTPUT_CO2_EQUIVALENT,
      BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  updateIaqSensorStatus();

  addMetaData(pressure);
  addMetaData(temperature);
  addMetaData(humidity);
  addMetaData(iaqAccuracy);
  addMetaData(staticIaq);
  return !fatalError;
}

std::vector<SHI::MeasurementBundle> SHI::BME680::readSensor() {
  uint32_t time_trigger = millis();
  static uint32_t lastStateTime = 0;

  SHI::hw->feedWatchdog();

  if (iaqSensor.run()) {  // If new data is available
    if (time_trigger - lastStateTime > 60000 && iaqSensor.iaqAccuracy >= 3) {
      lastStateTime = time_trigger;
      storeState();
    }
    updateIaqSensorStatus();
    std::vector<SHI::Measurement> res{
        temperature->measuredFloat(iaqSensor.temperature),
        humidity->measuredFloat(iaqSensor.humidity),
        pressure->measuredFloat(iaqSensor.pressure / 100.),
        iaqAccuracy->measuredInt(iaqSensor.iaqAccuracy),
        staticIaq->measuredFloat(iaqSensor.staticIaq)};
    if (statusMessage != SHI::STATUS_OK || fatalError) {
      res.push_back(getStatus());
    }
    return {{res, this}};
  }
  updateIaqSensorStatus();
  return {};
}
// Helper function definitions
void SHI::BME680::updateIaqSensorStatus(void) {
  statusMessage = SHI::STATUS_OK;
  fatalError = false;
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      String output = "BSEC error code : " + String(iaqSensor.status);
      SHI_LOGINFO(output.c_str());
      statusMessage = output.c_str();
      fatalError = true;
    } else {
      String output = "BSEC warning code : " + String(iaqSensor.status);
      SHI_LOGINFO(output.c_str());
      statusMessage = output.c_str();
      fatalError = false;
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      String output = "BME680 error code : " + String(iaqSensor.bme680Status);
      SHI_LOGINFO(output.c_str());
      statusMessage = output.c_str();
      fatalError = true;
    } else {
      String output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      SHI_LOGINFO(output.c_str());
      statusMessage = output.c_str();
      fatalError = false;
    }
  }
}
