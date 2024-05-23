/****************************************************************************
* Copyright (c) 2024 - Analog Devices Inc. All Rights Reserved.
* This software is proprietary & confidential to Analog Devices, Inc.
* and its licensors.
*******************************************************************************
*******************************************************************************
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.*/

#ifndef ADSD3500_INTERRUPT_NOTIFIER_H
#define ADSD3500_INTERRUPT_NOTIFIER_H

#include "adsd3500_util.h"
#include <iostream>
#include <memory>
#include <signal.h>
#include <sys/ioctl.h>
#include <vector>

class Adsd3500;

class Adsd3500InterruptNotifier {
  public:
    static Adsd3500InterruptNotifier &getInstance();
    static void signalEventHandler(int n, siginfo_t *info, void *unused);
    int enableInterrupts();
    int disableInterrupts();
    bool interruptsAvailable();
    void subscribeSensor(std::weak_ptr<Adsd3500> sensor);
    void unsubscribeSensor(std::weak_ptr<Adsd3500> sensor);

  private:
    Adsd3500InterruptNotifier() {}
    static std::vector<std::weak_ptr<Adsd3500>> m_sensors;
    bool m_interruptsAvailable = false;
};

#endif // ADSD3500_INTERRUPT_NOTIFIER_H
