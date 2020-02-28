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
#include <vector>

namespace SHI {

class BME680 : public SHI::Sensor {
 public:
  explicit BME680(TwoWire &i2cPort, bool primaryAddress = true)
      : Sensor("BME680"), primaryAddress(primaryAddress), wire(&i2cPort) {}
  std::vector<SHI::MeasurementBundle> readSensor() override;
  bool setupSensor() override;
  bool stopSensor() override { return true; }

 private:
  Bsec iaqSensor;
  Preferences bsecStatePrefs;
  void storeState();
  bool restoreState();
  void updateIaqSensorStatus(void);
  bool primaryAddress;
  TwoWire *wire;

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
