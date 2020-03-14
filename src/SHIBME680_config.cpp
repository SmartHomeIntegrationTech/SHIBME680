/*
 * Copyright (c) 2020 Karsten Becker All rights reserved.
 * Use of this source code is governed by a BSD-style
 * license that can be found in the LICENSE file.
 */

// WARNING, this is an automatically generated file!
// Don't change anything in here.
// Last update 2020-03-12

# include <iostream>
# include <string>


# include "SHIBME680.h"
// Configuration implementation for class SHI::BME680Config

SHI::BME680Config::BME680Config(const JsonObject &obj):
       useBus(obj["useBus"] | 0),
      primaryAddress(obj["primaryAddress"] | true),
      storeAndRestoreState(obj["storeAndRestoreState"] | true),
      sampleRate(obj["sampleRate"] | BSEC_SAMPLE_RATE_LP),
      minStableTime(obj["minStableTime"] | 60000)
  {}

void SHI::BME680Config::fillData(JsonObject &doc) const {
    doc["useBus"] = useBus;
  doc["primaryAddress"] = primaryAddress;
  doc["storeAndRestoreState"] = storeAndRestoreState;
  doc["sampleRate"] = sampleRate;
  doc["minStableTime"] = minStableTime;
}

int SHI::BME680Config::getExpectedCapacity() const {
  return JSON_OBJECT_SIZE(5);
}

