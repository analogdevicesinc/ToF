/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#include "adsd3500_interrupt_notifier.h"
#include "adsd3500_sensor.h"
#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#include <cstring>
#include <unistd.h>
#endif
#include <fcntl.h>
#include <sys/ioctl.h>

#define SIGETX 44
#define USER_TASK _IOW('A', 1, int32_t *)

std::vector<std::weak_ptr<Adsd3500Sensor>> Adsd3500InterruptNotifier::m_sensors;

Adsd3500InterruptNotifier &Adsd3500InterruptNotifier::getInstance() {
    static auto &&notifier = Adsd3500InterruptNotifier();
    return (notifier);
}

void Adsd3500InterruptNotifier::signalEventHandler(int n, siginfo_t *info,
                                                   void *unused) {
    if (n == SIGETX) {
        int signal_value = info->si_int;
        DLOG(INFO) << "Received signal " << info->si_int << " from kernel";

        for (auto sensor : m_sensors) {
            if (std::shared_ptr<Adsd3500Sensor> sptr = sensor.lock()) {
                sptr->adsd3500InterruptHandler(signal_value);
            }
        }
    }
}

aditof::Status Adsd3500InterruptNotifier::enableInterrupts() {
    // Subscribe to the ADSD3500 interrupt
    struct sigaction act;
    int32_t number;

    sigemptyset(&act.sa_mask);
    act.sa_flags = (SA_SIGINFO | SA_RESTART);
    act.sa_sigaction = Adsd3500InterruptNotifier::signalEventHandler;
    sigaction(SIGETX, &act, NULL);

    int debug_fd = ::open("/sys/kernel/debug/adsd3500/value", O_RDWR);
    if (debug_fd < 0) {
        LOG(WARNING) << "Failed to open the debug sysfs."
                     << "Interrupt support will not be available!";
        return aditof::Status::UNAVAILABLE;
    }

    if (ioctl(debug_fd, USER_TASK, (int32_t *)&number)) {
        LOG(WARNING) << "Failed to register the application process with the "
                        "kernel driver. IOCTL failed.";
        close(debug_fd);
    }

    return aditof::Status::OK;
}

aditof::Status Adsd3500InterruptNotifier::disableInterrupts() {
    return aditof::Status::OK;
}

void Adsd3500InterruptNotifier::subscribeSensor(
    std::weak_ptr<Adsd3500Sensor> sensor) {
    m_sensors.emplace_back(sensor);
}

void Adsd3500InterruptNotifier::unsubscribeSensor(
    std::weak_ptr<Adsd3500Sensor> sensor) {}