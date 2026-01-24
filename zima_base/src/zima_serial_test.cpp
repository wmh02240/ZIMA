/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/common/time.h"
#include "zima/hal/io/serial.h"
#include "zima/logger/logger.h"

using namespace zima;

void SerialReadTest() {
  ZINFO;
  Serial serial("/dev/ttyAMA0", 115200);
  ZINFO;
  if (!serial.Open()) {
    ZERROR;
    return;
  }

  Timer timer("Read duration", 3);
  while (!timer.TimeUp()) {
    serial.Read();
    // Time::SleepMSec(50);
  }
}

int main(int argc, char **argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  SerialReadTest();

  return 0;
}
