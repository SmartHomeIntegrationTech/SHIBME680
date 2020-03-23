/*
 * Copyright (c) 2020 Karsten Becker All rights reserved.
 * Use of this source code is governed by a BSD-style
 * license that can be found in the LICENSE file.
 */
#pragma once
#include <Preferences.h>
#include <SHISensor.h>
#include <bsec.h>

#include <memory>
#include <string>
#include <vector>

namespace SHI {

class BME680Config : public Configuration {
 public:
  explicit BME680Config(int useBus = 0, bool primaryAddress = true)
      : useBus(useBus), primaryAddress(primaryAddress) {}
  explicit BME680Config(const JsonObject &obj);
  void fillData(JsonObject &doc) const override;
  int useBus = 0;
  bool primaryAddress = true;
  bool storeAndRestoreState = true;
  float sampleRate = BSEC_SAMPLE_RATE_LP;
  int minStableTime = 60000;

 protected:
  int getExpectedCapacity() const override;
};

class BME680 : public SHI::Sensor {
 public:
  explicit BME680(const BME680Config &config);
  std::vector<SHI::MeasurementBundle> readSensor() override;
  bool setupSensor() override;
  bool stopSensor() override { return true; }
  const Configuration *getConfig() const override { return &config; }
  bool reconfigure(Configuration *newConfig);
  bool reconfigure();

 private:
  Bsec iaqSensor;
  Preferences bsecStatePrefs;
  void storeState();
  bool restoreState();
  void updateIaqSensorStatus(void);
  TwoWire *wire;
  BME680Config config;

  std::shared_ptr<SHI::MeasurementMetaData> temperature =
      std::make_shared<SHI::MeasurementMetaData>("Temperature", "°C",
                                                 SHI::SensorDataType::FLOAT);
  std::shared_ptr<SHI::MeasurementMetaData> humidity =
      std::make_shared<SHI::MeasurementMetaData>("Humidity", "%",
                                                 SHI::SensorDataType::FLOAT);
  std::shared_ptr<SHI::MeasurementMetaData> pressure =
      std::make_shared<SHI::MeasurementMetaData>("Pressure", "hPa",
                                                 SHI::SensorDataType::FLOAT);
  std::shared_ptr<SHI::MeasurementMetaData> staticIaq =
      std::make_shared<SHI::MeasurementMetaData>("StaticIaq", "",
                                                 SHI::SensorDataType::FLOAT);
  std::shared_ptr<SHI::MeasurementMetaData> iaqAccuracy =
      std::make_shared<SHI::MeasurementMetaData>("IaqAccuracy", "",
                                                 SHI::SensorDataType::INT);
};

}  // namespace SHI
