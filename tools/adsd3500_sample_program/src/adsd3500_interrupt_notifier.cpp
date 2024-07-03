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

#include <adsd3500_interrupt_notifier.h>
#include <adsd3500_util.h>

#include <algorithm>
#include <fcntl.h>
#include <memory>
#include <sys/ioctl.h>

#define SIGETX 44
#define USER_TASK _IOW('A', 1, int32_t *)

std::vector<std::weak_ptr<Adsd3500>> Adsd3500InterruptNotifier::m_sensors;

Adsd3500InterruptNotifier &Adsd3500InterruptNotifier::getInstance() {
    static auto &&notifier = Adsd3500InterruptNotifier();
    return notifier;
}

void Adsd3500InterruptNotifier::signalEventHandler(int n, siginfo_t *info,
                                                   void *unused) {
    if (n == SIGETX) {
        int signal_value = info->si_int;
        std::cout << "Received signal " << info->si_int << " from kernel"
                  << std::endl;

        for (auto sensor : m_sensors) {
            if (std::shared_ptr<Adsd3500> sptr = sensor.lock()) {
                sptr->HandleInterrupts(signal_value);
            }
        }
    }
}

int Adsd3500InterruptNotifier::enableInterrupts() {
    // Subscribe to the ADSD3500 interrupt
    struct sigaction act;
    int32_t number;

    sigemptyset(&act.sa_mask);
    act.sa_flags = (SA_SIGINFO | SA_RESTART);
    act.sa_sigaction = Adsd3500InterruptNotifier::signalEventHandler;
    sigaction(SIGETX, &act, NULL);

    m_interruptsAvailable = false;

    int debug_fd = ::open("/sys/kernel/debug/adsd3500/value", O_RDWR);
    if (debug_fd < 0) {
        std::cout << "Failed to open the debug sysfs. Interrupts support will "
                     "not be available!"
                  << std::endl;
        return -1;
    }

    if (ioctl(debug_fd, USER_TASK, (int32_t *)&number)) {
        std::cout << "Failed to register the application process with the "
                     "kernel driver. IOCTL failed."
                  << std::endl;
        close(debug_fd);
    }

    m_interruptsAvailable = true;

    return 0;
}

int Adsd3500InterruptNotifier::disableInterrupts() { return 0; }

bool Adsd3500InterruptNotifier::interruptsAvailable() {
    return m_interruptsAvailable;
}

void Adsd3500InterruptNotifier::subscribeSensor(
    std::weak_ptr<Adsd3500> sensor) {
    auto it = std::remove_if(
        m_sensors.begin(), m_sensors.end(),
        [](const std::weak_ptr<Adsd3500> &wp) { return wp.expired(); });
    m_sensors.erase(it, m_sensors.end());

    m_sensors.emplace_back(sensor);
}

void Adsd3500InterruptNotifier::unsubscribeSensor(
    std::weak_ptr<Adsd3500> sensor) {
    auto it = std::remove_if(m_sensors.begin(), m_sensors.end(),
                             [&sensor](const std::weak_ptr<Adsd3500> &wp) {
                                 if (auto sp = wp.lock()) {
                                     return sp == sensor.lock();
                                 }
                                 return false;
                             });
    m_sensors.erase(it, m_sensors.end());
}
