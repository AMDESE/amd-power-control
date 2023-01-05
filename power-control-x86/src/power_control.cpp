/*
// Copyright (c) 2018-2019 Intel Corporation
// Copyright (c) 2021 AMD Inc
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/


#include <sys/sysinfo.h>
#include <systemd/sd-journal.h>

#include <boost/asio/io_service.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <gpiod.hpp>

#include <phosphor-logging/log.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string_view>

#include <thread>
#define CHASSIS_INTRUDED         (0)
#define INTRUSION_POLL_INTERVAL  (1)

namespace power_control
{
static boost::asio::io_service io;
std::shared_ptr<sdbusplus::asio::connection> conn;

static std::string node = "0";

static std::shared_ptr<sdbusplus::asio::dbus_interface> hostIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> chassisIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> chassisSysIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> powerButtonIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> resetButtonIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> nmiButtonIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> osIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> idButtonIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> nmiOutIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> restartCauseIface;

const static constexpr int powerPulseTimeMs = 100;
const static constexpr int forceOffPulseTimeMs = 5000;
const static constexpr int resetPulseTimeMs = 100;
const static constexpr int powerCycleTimeMs = 5000;
const static constexpr int psPowerOKWatchdogTimeMs = 8000;
const static constexpr int psPowerOKRampDownTimeMs = 8000;
const static constexpr int gracefulPowerOffTimeMs = 90000;
const static constexpr int warmResetCheckTimeMs = 1000;
const static constexpr int powerOffSaveTimeMs = 7000;

const static std::filesystem::path powerControlDir = "/var/lib/power-control";
const static constexpr std::string_view powerStateFile = "power-state";

static bool nmiEnabled = true;

// Timers
// Time holding GPIOs asserted
static boost::asio::steady_timer gpioAssertTimer(io);
// Time between off and on during a power cycle
static boost::asio::steady_timer powerCycleTimer(io);
// Time OS gracefully powering off
static boost::asio::steady_timer gracefulPowerOffTimer(io);
// Time the warm reset check
static boost::asio::steady_timer warmResetCheckTimer(io);
// Time power supply power OK assertion on power-on
static boost::asio::steady_timer psPowerOKWatchdogTimer(io);
// Time power supply power OK ramp down on force power off
static boost::asio::steady_timer psPowerOKRampDownTimer(io);
// Time power-off state save for power loss tracking
static boost::asio::steady_timer powerStateSaveTimer(io);
// POH timer
static boost::asio::steady_timer pohCounterTimer(io);

// GPIO Lines and Event Descriptors
static gpiod::line psPowerOKLine;
static boost::asio::posix::stream_descriptor psPowerOKEvent(io);
static gpiod::line powerButtonLine;
static boost::asio::posix::stream_descriptor powerButtonEvent(io);
static gpiod::line resetButtonLine;
static boost::asio::posix::stream_descriptor resetButtonEvent(io);

enum class PowerState
{
    on,
    waitForPSPowerOK,
    off,
    transitionToOff,
    gracefulTransitionToOff,
    cycleOff,
    transitionToCycleOff,
    gracefulTransitionToCycleOff,
    checkForWarmReset,
};

static int getGPIOValue(const std::string& name)
{
    int value;
    gpiod::line gpioLine;
    // Find the GPIO line
    gpioLine = gpiod::find_line(name);
    if (!gpioLine)
    {
        std::cerr << "Can't find line: %s\n", name.c_str();
        return -1;
    }
    try
    {
        gpioLine.request({__FUNCTION__, gpiod::line_request::DIRECTION_INPUT});
    }
    catch (std::system_error& exc)
    {
        std::cerr << "Error setting gpio as Input: " << name << exc.what() << "\n";
    }

    try
    {
        value = gpioLine.get_value();
    }
    catch (std::system_error& exc)
    {
        std::cerr << "Error getting gpio value: " << name << exc.what() << "\n";
        return -1;
    }

    return value;
}

static PowerState powerState;
static std::string getPowerStateName(PowerState state)
{
    switch (state)
    {
        case PowerState::on:
            return "On";
            break;
        case PowerState::waitForPSPowerOK:
            return "Wait for Power Supply Power OK";
            break;
        case PowerState::off:
            return "Off";
            break;
        case PowerState::transitionToOff:
            return "Transition to Off";
            break;
        case PowerState::gracefulTransitionToOff:
            return "Graceful Transition to Off";
            break;
        case PowerState::cycleOff:
            return "Power Cycle Off";
            break;
        case PowerState::transitionToCycleOff:
            return "Transition to Power Cycle Off";
            break;
        case PowerState::gracefulTransitionToCycleOff:
            return "Graceful Transition to Power Cycle Off";
            break;
        case PowerState::checkForWarmReset:
            return "Check for Warm Reset";
            break;
        default:
            return "unknown state: " + std::to_string(static_cast<int>(state));
            break;
    }
}
static void logStateTransition(const PowerState state)
{
    std::string logMsg =
        "Host0: Moving to \"" + getPowerStateName(state) + "\" state";
    phosphor::logging::log<phosphor::logging::level::INFO>(
        logMsg.c_str(),
        phosphor::logging::entry("STATE=%s", getPowerStateName(state).c_str()),
        phosphor::logging::entry("HOST=0"));
}

enum class Event
{
    psPowerOKAssert,
    psPowerOKDeAssert,
    powerButtonPressed,
    resetButtonPressed,
    powerCycleTimerExpired,
    psPowerOKWatchdogTimerExpired,
    gracefulPowerOffTimerExpired,
    powerOnRequest,
    powerOffRequest,
    powerCycleRequest,
    resetRequest,
    gracefulPowerOffRequest,
    gracefulPowerCycleRequest,
    warmResetDetected,
    psPowerOKRampDownTimerExpired,
};
static std::string getEventName(Event event)
{
    switch (event)
    {
        case Event::psPowerOKAssert:
            return "power supply power OK assert";
            break;
        case Event::psPowerOKDeAssert:
            return "power supply power OK de-assert";
            break;
        case Event::powerButtonPressed:
            return "power button pressed";
            break;
        case Event::resetButtonPressed:
            return "reset button pressed";
            break;
        case Event::powerCycleTimerExpired:
            return "power cycle timer expired";
            break;
        case Event::psPowerOKWatchdogTimerExpired:
            return "power supply power OK watchdog timer expired";
            break;
        case Event::psPowerOKRampDownTimerExpired:
            return "power supply power OK RampDown timer expired";
            break;
        case Event::gracefulPowerOffTimerExpired:
            return "graceful power-off timer expired";
            break;
        case Event::powerOnRequest:
            return "power-on request";
            break;
        case Event::powerOffRequest:
            return "power-off request";
            break;
        case Event::powerCycleRequest:
            return "power-cycle request";
            break;
        case Event::resetRequest:
            return "reset request";
            break;
        case Event::gracefulPowerOffRequest:
            return "graceful power-off request";
            break;
        case Event::gracefulPowerCycleRequest:
            return "graceful power-cycle request";
            break;
        case Event::warmResetDetected:
            return "warm reset detected";
            break;
        default:
            return "unknown event: " + std::to_string(static_cast<int>(event));
            break;
    }
}
static void logEvent(const std::string_view stateHandler, const Event event)
{
    std::string logMsg{stateHandler};
    logMsg += ": " + getEventName(event) + " event received";
    phosphor::logging::log<phosphor::logging::level::INFO>(
        logMsg.c_str(),
        phosphor::logging::entry("EVENT=%s", getEventName(event).c_str()));
}

// Power state handlers
static void powerStateOn(const Event event);
static void powerStateWaitForPSPowerOK(const Event event);
static void powerStateOff(const Event event);
static void powerStateTransitionToOff(const Event event);
static void powerStateGracefulTransitionToOff(const Event event);
static void powerStateCycleOff(const Event event);
static void powerStateTransitionToCycleOff(const Event event);
static void powerStateGracefulTransitionToCycleOff(const Event event);
static void powerStateCheckForWarmReset(const Event event);

static std::function<void(const Event)> getPowerStateHandler(PowerState state)
{
    switch (state)
    {
        case PowerState::on:
            return powerStateOn;
            break;
        case PowerState::waitForPSPowerOK:
            return powerStateWaitForPSPowerOK;
            break;
        case PowerState::off:
            return powerStateOff;
            break;
        case PowerState::transitionToOff:
            return powerStateTransitionToOff;
            break;
        case PowerState::gracefulTransitionToOff:
            return powerStateGracefulTransitionToOff;
            break;
        case PowerState::cycleOff:
            return powerStateCycleOff;
            break;
        case PowerState::transitionToCycleOff:
            return powerStateTransitionToCycleOff;
            break;
        case PowerState::gracefulTransitionToCycleOff:
            return powerStateGracefulTransitionToCycleOff;
            break;
        case PowerState::checkForWarmReset:
            return powerStateCheckForWarmReset;
            break;
        default:
            return std::function<void(const Event)>{};
            break;
    }
};

static void sendPowerControlEvent(const Event event)
{
    std::function<void(const Event)> handler = getPowerStateHandler(powerState);
    if (handler == nullptr)
    {
        std::cerr << "Failed to find handler for power state: "
                  << static_cast<int>(powerState) << "\n";
        return;
    }
    handler(event);
}

static uint64_t getCurrentTimeMs()
{
    struct timespec time = {};

    if (clock_gettime(CLOCK_REALTIME, &time) < 0)
    {
        return 0;
    }
    uint64_t currentTimeMs = static_cast<uint64_t>(time.tv_sec) * 1000;
    currentTimeMs += static_cast<uint64_t>(time.tv_nsec) / 1000 / 1000;

    return currentTimeMs;
}

static constexpr std::string_view getHostState(const PowerState state)
{
    switch (state)
    {
        case PowerState::on:
        case PowerState::transitionToOff:
        case PowerState::gracefulTransitionToOff:
        case PowerState::transitionToCycleOff:
        case PowerState::gracefulTransitionToCycleOff:
            return "xyz.openbmc_project.State.Host.HostState.Running";
            break;
        case PowerState::waitForPSPowerOK:
        case PowerState::off:
        case PowerState::cycleOff:
        case PowerState::checkForWarmReset:
            return "xyz.openbmc_project.State.Host.HostState.Off";
            break;
        default:
            return "";
            break;
    }
};
static constexpr std::string_view getChassisState(const PowerState state)
{
    switch (state)
    {
        case PowerState::on:
        case PowerState::transitionToOff:
        case PowerState::gracefulTransitionToOff:
        case PowerState::transitionToCycleOff:
        case PowerState::gracefulTransitionToCycleOff:
        case PowerState::checkForWarmReset:
            return "xyz.openbmc_project.State.Chassis.PowerState.On";
            break;
        case PowerState::waitForPSPowerOK:
        case PowerState::off:
        case PowerState::cycleOff:
            return "xyz.openbmc_project.State.Chassis.PowerState.Off";
            break;
        default:
            return "";
            break;
    }
};
static void savePowerState(const PowerState state)
{
    powerStateSaveTimer.expires_after(
        std::chrono::milliseconds(powerOffSaveTimeMs));
    powerStateSaveTimer.async_wait([state](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::cerr << "Power-state save async_wait failed: "
                          << ec.message() << "\n";
            }
            return;
        }
        std::ofstream powerStateStream(powerControlDir / powerStateFile);
        powerStateStream << getChassisState(state);
    });
}
static void setPowerState(const PowerState state)
{
    powerState = state;
    logStateTransition(state);

    hostIface->set_property("CurrentHostState",
                            std::string(getHostState(powerState)));

    chassisIface->set_property("CurrentPowerState",
                               std::string(getChassisState(powerState)));
    chassisIface->set_property("LastStateChangeTime", getCurrentTimeMs());

    // Save the power state for the restore policy
    savePowerState(state);
}

enum class RestartCause
{
    command,
    resetButton,
    powerButton,
    watchdog,
    powerPolicyOn,
    powerPolicyRestore,
    softReset,
};
static boost::container::flat_set<RestartCause> causeSet;
static std::string getRestartCause(RestartCause cause)
{
    switch (cause)
    {
        case RestartCause::command:
            return "xyz.openbmc_project.State.Host.RestartCause.IpmiCommand";
            break;
        case RestartCause::resetButton:
            return "xyz.openbmc_project.State.Host.RestartCause.ResetButton";
            break;
        case RestartCause::powerButton:
            return "xyz.openbmc_project.State.Host.RestartCause.PowerButton";
            break;
        case RestartCause::watchdog:
            return "xyz.openbmc_project.State.Host.RestartCause.WatchdogTimer";
            break;
        case RestartCause::powerPolicyOn:
            return "xyz.openbmc_project.State.Host.RestartCause."
                   "PowerPolicyAlwaysOn";
            break;
        case RestartCause::powerPolicyRestore:
            return "xyz.openbmc_project.State.Host.RestartCause."
                   "PowerPolicyPreviousState";
            break;
        case RestartCause::softReset:
            return "xyz.openbmc_project.State.Host.RestartCause.SoftReset";
            break;
        default:
            return "xyz.openbmc_project.State.Host.RestartCause.Unknown";
            break;
    }
}
static void addRestartCause(const RestartCause cause)
{
    // Add this to the set of causes for this restart
    causeSet.insert(cause);
}
static void clearRestartCause()
{
    // Clear the set for the next restart
    causeSet.clear();
}
static void setRestartCauseProperty(const std::string& cause)
{
    std::cerr << "RestartCause set to " << cause << "\n";
    restartCauseIface->set_property("RestartCause", cause);
}

static void setRestartCause()
{
    // Determine the actual restart cause based on the set of causes
    std::string restartCause =
        "xyz.openbmc_project.State.Host.RestartCause.Unknown";
    if (causeSet.contains(RestartCause::watchdog))
    {
        restartCause = getRestartCause(RestartCause::watchdog);
    }
    else if (causeSet.contains(RestartCause::command))
    {
        restartCause = getRestartCause(RestartCause::command);
    }
    else if (causeSet.contains(RestartCause::resetButton))
    {
        restartCause = getRestartCause(RestartCause::resetButton);
    }
    else if (causeSet.contains(RestartCause::powerButton))
    {
        restartCause = getRestartCause(RestartCause::powerButton);
    }
    else if (causeSet.contains(RestartCause::powerPolicyOn))
    {
        restartCause = getRestartCause(RestartCause::powerPolicyOn);
    }
    else if (causeSet.contains(RestartCause::powerPolicyRestore))
    {
        restartCause = getRestartCause(RestartCause::powerPolicyRestore);
    }
    else if (causeSet.contains(RestartCause::softReset))
    {
        restartCause = getRestartCause(RestartCause::softReset);
    }

    setRestartCauseProperty(restartCause);
}

static void psPowerOKFailedLog()
{
    sd_journal_send(
        "MESSAGE=PowerControl: power supply power good failed to assert",
        "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
        "OpenBMC.0.1.PowerSupplyPowerGoodFailed", "REDFISH_MESSAGE_ARGS=%d",
        psPowerOKWatchdogTimeMs, NULL);
}

static void psPowerOKRampDownFailedLog()
{
    sd_journal_send(
        "MESSAGE=PowerControl: power supply power good failed to rampdown",
        "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
        "OpenBMC.0.1.PowerSupplyPowerGoodRampDownFailed", "REDFISH_MESSAGE_ARGS=%d",
        psPowerOKRampDownTimeMs, NULL);
}

static void powerRestorePolicyLog()
{
    sd_journal_send("MESSAGE=PowerControl: power restore policy applied",
                    "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.PowerRestorePolicyApplied", NULL);
}

static void powerButtonPressLog()
{
    sd_journal_send("MESSAGE=PowerControl: power button pressed", "PRIORITY=%i",
                    LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.PowerButtonPressed", NULL);
}

static void resetButtonPressLog()
{
    sd_journal_send("MESSAGE=PowerControl: reset button pressed", "PRIORITY=%i",
                    LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.ResetButtonPressed", NULL);
}

static void nmiButtonPressLog()
{
    sd_journal_send("MESSAGE=PowerControl: NMI button pressed", "PRIORITY=%i",
                    LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.NMIButtonPressed", NULL);
}

static void nmiDiagIntLog()
{
    sd_journal_send("MESSAGE=PowerControl: NMI Diagnostic Interrupt",
                    "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.NMIDiagnosticInterrupt", NULL);
}

static int initializePowerStateStorage()
{
    // create the power control directory if it doesn't exist
    std::error_code ec;
    if (!(std::filesystem::create_directories(powerControlDir, ec)))
    {
        if (ec.value() != 0)
        {
            std::cerr << "failed to create " << powerControlDir << ": "
                      << ec.message() << "\n";
            return -1;
        }
    }
    // Create the power state file if it doesn't exist
    if (!std::filesystem::exists(powerControlDir / powerStateFile))
    {
        std::ofstream powerStateStream(powerControlDir / powerStateFile);
        powerStateStream << getChassisState(powerState);
    }
    return 0;
}

static bool wasPowerDropped()
{
    std::ifstream powerStateStream(powerControlDir / powerStateFile);
    if (!powerStateStream.is_open())
    {
        std::cerr << "Failed to open power state file\n";
        return false;
    }

    std::string state;
    std::getline(powerStateStream, state);
    return state == "xyz.openbmc_project.State.Chassis.PowerState.On";
}

static void powerRestorePolicyCheck()
{
    // Check restore policy on DBus
    conn->async_method_call(
        [](boost::system::error_code ec,
                const std::variant<std::string>& restorepolicy) {
        if (ec)
        {
            std::cerr << "Error read restore policy\n";
            return;
        }
        const std::string* policy =
                std::get_if<std::string>(&restorepolicy);
        if (policy == nullptr)
        {
            std::cerr << "Unable to read restore policy\n";
            return;
        }

        if (policy->compare("None") == 0)
        {
            return;
        }
        else
        {
            std::cerr << "Invoking Restore Policy: " << policy->c_str() << " \n";

            sd_journal_send("MESSAGE=PowerControl: power restore policy applied",
                    "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.PowerRestorePolicyApplied", NULL);

            if (policy->compare("xyz.openbmc_project.Control.Power.RestorePolicy.Policy.AlwaysOn") == 0)
            {
                sendPowerControlEvent(Event::powerOnRequest);
                setRestartCauseProperty(getRestartCause(RestartCause::powerPolicyOn));
            }
            else if (policy->compare("xyz.openbmc_project.Control.Power.RestorePolicy.Policy.Restore") == 0)
            {
                if (wasPowerDropped())
                {
                    std::cerr << "Power was dropped, restoring Host On state\n";
                    sendPowerControlEvent(Event::powerOnRequest);
                    setRestartCauseProperty(getRestartCause(RestartCause::powerPolicyRestore));
                }
                else
                {
                    std::cerr << "No power drop, restoring Host Off state\n";
                }
            }
            // We're done with the previous power state for the restore policy, so store
            // the current state
            savePowerState(powerState);
        }
    },
    "xyz.openbmc_project.Settings",
    "/xyz/openbmc_project/control/host0/power_restore_policy",
    "org.freedesktop.DBus.Properties", "Get",
    "xyz.openbmc_project.Control.Power.RestorePolicy", "PowerRestorePolicy");
}

static bool requestGPIOEvents(
    const std::string& name, const std::function<void()>& handler,
    gpiod::line& gpioLine,
    boost::asio::posix::stream_descriptor& gpioEventDescriptor)
{
    // Find the GPIO line
    gpioLine = gpiod::find_line(name);
    if (!gpioLine)
    {
        std::cerr << "Failed to find the " << name << " line\n";
        return false;
    }

    try
    {
        gpioLine.request(
            {"power-control", gpiod::line_request::EVENT_BOTH_EDGES});
    }
    catch (std::exception& exc)
    {
        std::cerr << "Failed to request events for " << name << exc.what() << "\n";
        return false;
    }

    int gpioLineFd = gpioLine.event_get_fd();
    if (gpioLineFd < 0)
    {
        std::cerr << "Failed to get " << name << " fd\n";
        return false;
    }

    gpioEventDescriptor.assign(gpioLineFd);

    gpioEventDescriptor.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [&name, handler](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << name << " fd handler error: " << ec.message()
                          << "\n";
                // TODO: throw here to force power-control to restart?
                return;
            }
            handler();
        });

    return true;
}

static bool setGPIOOutput(const std::string& name, const int value,
                          gpiod::line& gpioLine)
{
    // Find the GPIO line
    gpioLine = gpiod::find_line(name);
    if (!gpioLine)
    {
        std::cerr << "Failed to find the " << name << " line.\n";
        return false;
    }

    try
    {
        gpioLine.request({__FUNCTION__, gpiod::line_request::DIRECTION_OUTPUT});
    }
    catch (std::system_error& exc)
    {
        std::cerr << "Error setting gpio as Output: " << name << exc.what() << "\n";
    }

    try
    {
        // Request GPIO output to specified value
        gpioLine.set_value(value);
    }
    catch (std::exception& exc)
    {
        std::cerr << "Failed to set value for " << name << exc.what()<< "\n";
        return false;
    }


    std::cerr << name << " set to " << std::to_string(value) << "\n";

    return true;
}

static int setGPIOOutputForMs(const std::string& name, const int value,
                              const int durationMs)
{
    // No mask set, so request and set the GPIO normally
    gpiod::line gpioLine;
    if (!setGPIOOutput(name, value, gpioLine))
    {
        return -1;
    }
    gpioAssertTimer.expires_after(std::chrono::milliseconds(durationMs));
    gpioAssertTimer.async_wait(
        [gpioLine, value, name](const boost::system::error_code ec) {
            // Set the GPIO line back to the opposite value
            gpioLine.set_value(!value);
            std::cerr << name << " released\n";
            if (ec)
            {
                // operation_aborted is expected if timer is canceled before
                // completion.
                if (ec != boost::asio::error::operation_aborted)
                {
                    std::cerr << name << " async_wait failed: " << ec.message()
                              << "\n";
                }
            }
        });
    return 0;
}

static void powerOn()
{
    setGPIOOutputForMs("ASSERT_PWR_BTN_L", 0, powerPulseTimeMs);
}

static void gracefulPowerOff()
{
    setGPIOOutputForMs("ASSERT_PWR_BTN_L", 0, powerPulseTimeMs);
}

static void forcePowerOff()
{
    setGPIOOutputForMs("ASSERT_PWR_BTN_L", 0, forceOffPulseTimeMs);
    return;
}

static void reset()
{
    setGPIOOutputForMs("ASSERT_RST_BTN_L", 0, resetPulseTimeMs);
}

static void gracefulPowerOffTimerStart()
{
    std::cerr << "Graceful power-off timer started\n";
    gracefulPowerOffTimer.expires_after(
        std::chrono::milliseconds(gracefulPowerOffTimeMs));
    gracefulPowerOffTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::cerr << "Graceful power-off async_wait failed: "
                          << ec.message() << "\n";
            }
            std::cerr << "Graceful power-off timer canceled\n";
            return;
        }
        std::cerr << "Graceful power-off timer completed\n";
        sendPowerControlEvent(Event::gracefulPowerOffTimerExpired);
    });
}

static void powerCycleTimerStart()
{
    std::cerr << "Power-cycle timer started\n";
    powerCycleTimer.expires_after(std::chrono::milliseconds(powerCycleTimeMs));
    powerCycleTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::cerr << "Power-cycle async_wait failed: " << ec.message()
                          << "\n";
            }
            std::cerr << "Power-cycle timer canceled\n";
            return;
        }
        std::cerr << "Power-cycle timer completed\n";
        sendPowerControlEvent(Event::powerCycleTimerExpired);
    });
}

static void psPowerOKWatchdogTimerStart()
{
    std::cerr << "power supply power OK watchdog timer started\n";
    psPowerOKWatchdogTimer.expires_after(
        std::chrono::milliseconds(psPowerOKWatchdogTimeMs));
    psPowerOKWatchdogTimer.async_wait(
        [](const boost::system::error_code ec) {
            if (ec)
            {
                // operation_aborted is expected if timer is canceled before
                // completion.
                if (ec != boost::asio::error::operation_aborted)
                {
                    std::cerr
                        << "power supply power OK watchdog async_wait failed: "
                        << ec.message() << "\n";
                }
                std::cerr << "power supply power OK watchdog timer canceled\n";
                return;
            }
            std::cerr << "power supply power OK watchdog timer expired\n";
            sendPowerControlEvent(Event::psPowerOKWatchdogTimerExpired);
        });
}

static void psPowerOKRampDownTimerStart()
{
    std::cerr << "power supply power OK RampDown timer started\n";
    psPowerOKRampDownTimer.expires_after(
        std::chrono::milliseconds(psPowerOKRampDownTimeMs));
    psPowerOKRampDownTimer.async_wait(
        [](const boost::system::error_code ec) {
            if (ec)
            {
                // operation_aborted is expected if timer is canceled before
                // completion.
                if (ec != boost::asio::error::operation_aborted)
                {
                    std::cerr
                        << "power supply power OK RampDown async_wait failed: "
                        << ec.message() << "\n";
                }
                std::cerr << "power supply power OK RampDown timer canceled\n";
                return;
            }
            std::cerr << "power supply power OK RampDown timer expired\n";
            sendPowerControlEvent(Event::psPowerOKRampDownTimerExpired);
        });
}

static void warmResetCheckTimerStart()
{
    std::cerr << "Warm reset check timer started\n";
    warmResetCheckTimer.expires_after(
        std::chrono::milliseconds(warmResetCheckTimeMs));
    warmResetCheckTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::cerr << "Warm reset check async_wait failed: "
                          << ec.message() << "\n";
            }
            std::cerr << "Warm reset check timer canceled\n";
            return;
        }
        std::cerr << "Warm reset check timer completed\n";
        sendPowerControlEvent(Event::warmResetDetected);
    });
}

static void pohCounterTimerStart()
{
    std::cerr << "POH timer started\n";
    // Set the time-out as 1 hour, to align with POH command in ipmid
    pohCounterTimer.expires_after(std::chrono::hours(1));
    pohCounterTimer.async_wait([](const boost::system::error_code& ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::cerr << "POH timer async_wait failed: " << ec.message()
                          << "\n";
            }
            std::cerr << "POH timer canceled\n";
            return;
        }

        if (getHostState(powerState) !=
            "xyz.openbmc_project.State.Host.HostState.Running")
        {
            return;
        }

        conn->async_method_call(
            [](boost::system::error_code ec,
               const std::variant<uint32_t>& pohCounterProperty) {
                if (ec)
                {
                    std::cerr << "error to get poh counter\n";
                    return;
                }
                const uint32_t* pohCounter =
                    std::get_if<uint32_t>(&pohCounterProperty);
                if (pohCounter == nullptr)
                {
                    std::cerr << "unable to read poh counter\n";
                    return;
                }

                conn->async_method_call(
                    [](boost::system::error_code ec) {
                        if (ec)
                        {
                            std::cerr << "failed to set poh counter\n";
                        }
                    },
                    "xyz.openbmc_project.Settings",
                    "/xyz/openbmc_project/state/chassis0",
                    "org.freedesktop.DBus.Properties", "Set",
                    "xyz.openbmc_project.State.PowerOnHours", "POHCounter",
                    std::variant<uint32_t>(*pohCounter + 1));
            },
            "xyz.openbmc_project.Settings",
            "/xyz/openbmc_project/state/chassis0",
            "org.freedesktop.DBus.Properties", "Get",
            "xyz.openbmc_project.State.PowerOnHours", "POHCounter");

        pohCounterTimerStart();
    });
}

static void currentHostStateMonitor()
{
    if (getHostState(powerState) ==
        "xyz.openbmc_project.State.Host.HostState.Running")
    {
        pohCounterTimerStart();
        // Clear the restart cause set for the next restart
        clearRestartCause();
    }
    else
    {
        pohCounterTimer.cancel();
        // Set the restart cause set for this restart
        setRestartCause();
    }

    static auto match = sdbusplus::bus::match::match(
        *conn,
        "type='signal',member='PropertiesChanged', "
        "interface='org.freedesktop.DBus.Properties', "
        "arg0='xyz.openbmc_project.State.Host'",
        [](sdbusplus::message::message& message) {
            std::string intfName;
            std::map<std::string, std::variant<std::string>> properties;

            try
            {
                message.read(intfName, properties);
            }
            catch (std::exception& e)
            {
                std::cerr << "Unable to read host state\n";
                return;
            }
            if (properties.empty())
            {
                std::cerr << "ERROR: Empty PropertiesChanged signal received\n";
                return;
            }

            // We only want to check for CurrentHostState
            if (properties.begin()->first != "CurrentHostState")
            {
                return;
            }
            std::string* currentHostState =
                std::get_if<std::string>(&(properties.begin()->second));
            if (currentHostState == nullptr)
            {
                std::cerr << properties.begin()->first << " property invalid\n";
                return;
            }

            if (*currentHostState ==
                "xyz.openbmc_project.State.Host.HostState.Running")
            {
                pohCounterTimerStart();
                // Clear the restart cause set for the next restart
                clearRestartCause();
                sd_journal_send("MESSAGE=Host system DC power is on",
                                "PRIORITY=%i", LOG_INFO,
                                "REDFISH_MESSAGE_ID=%s",
                                "OpenBMC.0.1.DCPowerOn", NULL);
            }
            else
            {
                pohCounterTimer.cancel();
                // POST_COMPLETE GPIO event is not working in some platforms
                // when power state is changed to OFF. This resulted in
                // 'OperatingSystemState' to stay at 'Standby', even though
                // system is OFF. Set 'OperatingSystemState' to 'Inactive'
                // if HostState is trurned to OFF.
                osIface->set_property("OperatingSystemState",
                                      std::string("Inactive"));

                // Set the restart cause set for this restart
                setRestartCause();
                sd_journal_send("MESSAGE=Host system DC power is off",
                                "PRIORITY=%i", LOG_INFO,
                                "REDFISH_MESSAGE_ID=%s",
                                "OpenBMC.0.1.DCPowerOff", NULL);
            }
        });
}

static void powerStateOn(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            setPowerState(PowerState::off);
            break;
        case Event::powerButtonPressed:
            setPowerState(PowerState::gracefulTransitionToOff);
            gracefulPowerOffTimerStart();
            break;
        case Event::resetButtonPressed:
            setPowerState(PowerState::checkForWarmReset);
            warmResetCheckTimerStart();
            break;
        case Event::powerOffRequest:
            psPowerOKRampDownTimerStart();
            setPowerState(PowerState::transitionToOff);
            forcePowerOff();
            break;
        case Event::gracefulPowerOffRequest:
            setPowerState(PowerState::gracefulTransitionToOff);
            gracefulPowerOffTimerStart();
            gracefulPowerOff();
            break;
        case Event::powerCycleRequest:
            setPowerState(PowerState::transitionToCycleOff);
            forcePowerOff();
            break;
        case Event::gracefulPowerCycleRequest:
            setPowerState(PowerState::gracefulTransitionToCycleOff);
            gracefulPowerOffTimerStart();
            gracefulPowerOff();
            break;
        case Event::resetRequest:
            reset();
            break;
        default:
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "No action taken.");
            break;
    }
}

static void powerStateWaitForPSPowerOK(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKAssert:
            // Cancel any GPIO assertions held during the transition
            gpioAssertTimer.cancel();
            psPowerOKWatchdogTimer.cancel();
            setPowerState(PowerState::on);
            break;
        case Event::psPowerOKWatchdogTimerExpired:
            setPowerState(PowerState::off);
            psPowerOKFailedLog();
            break;
        default:
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "No action taken.");
            break;
    }
}

static void powerStateOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKAssert:
            setPowerState(PowerState::on);
            break;
        case Event::powerButtonPressed:
            psPowerOKWatchdogTimerStart();
            setPowerState(PowerState::waitForPSPowerOK);
            break;
        case Event::powerOnRequest:
            psPowerOKWatchdogTimerStart();
            setPowerState(PowerState::waitForPSPowerOK);
            powerOn();
            break;
        default:
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "No action taken.");
            break;
    }
}

static void powerStateTransitionToOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            // Cancel any GPIO assertions held during the transition
            gpioAssertTimer.cancel();
            psPowerOKRampDownTimer.cancel();
            setPowerState(PowerState::off);
            break;
        case Event::psPowerOKRampDownTimerExpired:
            psPowerOKRampDownFailedLog();
            setPowerState(PowerState::on);
            break;
        default:
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "No action taken.");
            break;
    }
}

static void powerStateGracefulTransitionToOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::off);
            break;
        case Event::gracefulPowerOffTimerExpired:
            setPowerState(PowerState::on);
            break;
        default:
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "No action taken.");
            break;
    }
}

static void powerStateCycleOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKAssert:
            powerCycleTimer.cancel();
            setPowerState(PowerState::on);
            break;
        case Event::psPowerOKDeAssert:
            powerCycleTimer.cancel();
            setPowerState(PowerState::off);
            break;
        case Event::powerButtonPressed:
            powerCycleTimer.cancel();
            psPowerOKWatchdogTimerStart();
            setPowerState(PowerState::waitForPSPowerOK);
            break;
        case Event::powerCycleTimerExpired:
            psPowerOKWatchdogTimerStart();
            setPowerState(PowerState::waitForPSPowerOK);
            powerOn();
            break;
        default:
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "No action taken.");
            break;
    }
}

static void powerStateTransitionToCycleOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            // Cancel any GPIO assertions held during the transition
            gpioAssertTimer.cancel();
            setPowerState(PowerState::cycleOff);
            powerCycleTimerStart();
            break;
        default:
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "No action taken.");
            break;
    }
}

static void powerStateGracefulTransitionToCycleOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::cycleOff);
            powerCycleTimerStart();
            break;
        case Event::gracefulPowerOffTimerExpired:
            setPowerState(PowerState::on);
            break;
        default:
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "No action taken.");
            break;
    }
}

static void powerStateCheckForWarmReset(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::warmResetDetected:
            setPowerState(PowerState::on);
            break;
        case Event::psPowerOKDeAssert:
            warmResetCheckTimer.cancel();
            setPowerState(PowerState::off);
            break;
        default:
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "No action taken.");
            break;
    }
}

static void psPowerOKHandler()
{
    gpiod::line_event gpioLineEvent = psPowerOKLine.event_read();

    Event powerControlEvent =
        gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE
            ? Event::psPowerOKAssert
            : Event::psPowerOKDeAssert;

    sendPowerControlEvent(powerControlEvent);

    psPowerOKEvent.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << "power supply power OK handler error: "
                          << ec.message() << "\n";
                return;
            }
            psPowerOKHandler();
        });
}

static void powerButtonHandler()
{
    gpiod::line_event gpioLineEvent = powerButtonLine.event_read();

    if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE)
    {
        powerButtonPressLog();
        powerButtonIface->set_property("ButtonPressed", true);

        sendPowerControlEvent(Event::powerButtonPressed);
        addRestartCause(RestartCause::powerButton);

    }
    else if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE)
    {
        powerButtonIface->set_property("ButtonPressed", false);
    }
    powerButtonEvent.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << "power button handler error: " << ec.message()
                          << "\n";
                return;
            }
            powerButtonHandler();
        });
}

static void resetButtonHandler()
{
    gpiod::line_event gpioLineEvent = resetButtonLine.event_read();

    if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE)
    {
        resetButtonPressLog();
        resetButtonIface->set_property("ButtonPressed", true);

        sendPowerControlEvent(Event::resetButtonPressed);
        addRestartCause(RestartCause::resetButton);

    }
    else if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE)
    {
        resetButtonIface->set_property("ButtonPressed", false);
    }
    resetButtonEvent.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << "reset button handler error: " << ec.message()
                          << "\n";
                return;
            }
            resetButtonHandler();
        });
}

static constexpr auto systemdBusname = "org.freedesktop.systemd1";
static constexpr auto systemdPath = "/org/freedesktop/systemd1";
static constexpr auto systemdInterface = "org.freedesktop.systemd1.Manager";
static constexpr auto systemTargetName = "chassis-system-reset.target";

void systemReset()
{
    conn->async_method_call(
        [](boost::system::error_code ec) {
            if (ec)
            {
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    "Failed to call chassis system reset",
                    phosphor::logging::entry("ERR=%s", ec.message().c_str()));
            }
        },
        systemdBusname, systemdPath, systemdInterface, "StartUnit",
        systemTargetName, "replace");
}
static void chassisIntrusionMonitor()
{
    /* AST2500/2600 chass intrusion controller dev path */
    const std::string driver_path = "/sys/devices/platform/ahb/ahb:apb/1e6ef010.chassis/hwmon/";
    const std::string hwmon_filename = "/intrusion0_alarm";
    std::string hwmon_folder;

    /* we don't know hwmon folder's number in path, search platform.
       driver creates only one hwmon folder, break on first result */
    if (std::filesystem::exists (driver_path))
    {
        for (const auto& folder : std::filesystem::directory_iterator(driver_path))
        {
            hwmon_folder = folder.path();
            break;
        }

        /* check for intrusion sysfs file, 0 -> assert, 1 -> deassert */
        if (std::filesystem::exists (hwmon_folder + hwmon_filename))
        {
            /* create polling thread, interval 1 sec (phosphor-hwmon default interval) */
            std::thread([hwmon_folder, hwmon_filename](){
                std::ifstream hfile;
                unsigned int alarm_status;
                unsigned int prev_status = 0xFF;
                while(1) {
                    std::this_thread::sleep_for(std::chrono::seconds(INTRUSION_POLL_INTERVAL));
                    hfile.open(hwmon_folder + hwmon_filename);
                    hfile >> alarm_status;
                    hfile.close();
                    if(prev_status != alarm_status)
                    {
                        prev_status = alarm_status;
                        if(alarm_status == CHASSIS_INTRUDED)
                        {
                            sd_journal_send("MESSAGE=Chassis Intrusion Detected. Current state: Open",
                                    "PRIORITY=%i", LOG_WARNING, "REDFISH_MESSAGE_ID=%s",
                                    "OpenBMC.0.1.ChassisIntrusionDetected", NULL);
                        }
                        else
                        {
                            sd_journal_send("MESSAGE=Chassis Intrusion Recovered. Current state: Closed",
                                    "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                                    "OpenBMC.0.1.ChassisIntrusionReset", NULL);
                        }
                    }
                }
            }).detach();
        }
    }
}

static void nmiSetEnableProperty(bool value)
{
    conn->async_method_call(
        [](boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << "failed to set NMI source\n";
            }
        },
        "xyz.openbmc_project.Settings",
        "/xyz/openbmc_project/Chassis/Control/NMISource",
        "org.freedesktop.DBus.Properties", "Set",
        "xyz.openbmc_project.Chassis.Control.NMISource", "Enabled",
        std::variant<bool>{value});
}

static void nmiReset()
{
    const static constexpr int nmiOutPulseTimeMs = 200;
    setGPIOOutputForMs("ASSERT_NMI_BTN_L", 0, nmiOutPulseTimeMs);

    // log to redfish
    nmiDiagIntLog();
    std::cerr << "NMI out action completed\n";
    // reset Enable Property
    nmiSetEnableProperty(false);
}

static void nmiSourcePropertyMonitor(void)
{
    std::cerr << " NMI Source Property Monitor \n";

    static std::unique_ptr<sdbusplus::bus::match::match> nmiSourceMatch =
        std::make_unique<sdbusplus::bus::match::match>(
            *conn,
            "type='signal',interface='org.freedesktop.DBus.Properties',"
            "member='PropertiesChanged',arg0namespace='xyz.openbmc_project."
            "Chassis.Control."
            "NMISource'",
            [](sdbusplus::message::message& msg) {
                std::string interfaceName;
                boost::container::flat_map<std::string,
                                           std::variant<bool, std::string>>
                    propertiesChanged;
                std::string state;
                bool value = true;
                try
                {
                    msg.read(interfaceName, propertiesChanged);
                    if (propertiesChanged.begin()->first == "Enabled")
                    {
                        value =
                            std::get<bool>(propertiesChanged.begin()->second);
                        std::cerr
                            << " NMI Enabled propertiesChanged value: " << value
                            << "\n";
                        nmiEnabled = value;
                        if (nmiEnabled)
                        {
                            nmiReset();
                        }
                    }
                }
                catch (std::exception& e)
                {
                    std::cerr << "Unable to read NMI source\n";
                    return;
                }
            });
}

static void setNmiSource()
{
    conn->async_method_call(
        [](boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << "failed to set NMI source\n";
            }
        },
        "xyz.openbmc_project.Settings",
        "/xyz/openbmc_project/Chassis/Control/NMISource",
        "org.freedesktop.DBus.Properties", "Set",
        "xyz.openbmc_project.Chassis.Control.NMISource", "BMCSource",
        std::variant<std::string>{"xyz.openbmc_project.Chassis.Control."
                                  "NMISource.BMCSourceSignal.FpBtn"});
    // set Enable Property
    nmiSetEnableProperty(true);
}

} // namespace power_control

int main(int argc, char* argv[])
{
    std::cerr << "Start Chassis power control service...\n";

    if (system("/usr/sbin/dimm-info.sh") != 0)
        std::cerr << "Error calling dimm-info.sh in Chassis power control service \n";
    power_control::conn =
        std::make_shared<sdbusplus::asio::connection>(power_control::io);

    // Request all the dbus names
    power_control::conn->request_name("xyz.openbmc_project.State.Host");
    power_control::conn->request_name("xyz.openbmc_project.State.Chassis");
    power_control::conn->request_name("xyz.openbmc_project.State.OperatingSystem");
    power_control::conn->request_name("xyz.openbmc_project.Chassis.Buttons");
    power_control::conn->request_name("xyz.openbmc_project.Control.Host.NMI");
    power_control::conn->request_name("xyz.openbmc_project.Control.Host.RestartCause");

    // Request PS_PWROK GPIO events
    power_control::requestGPIOEvents("MON_PWR_GOOD", power_control::psPowerOKHandler, power_control::psPowerOKLine, power_control::psPowerOKEvent);

    // Request POWER_BUTTON GPIO events
    power_control::requestGPIOEvents("MON_PWR_BTN_L", power_control::powerButtonHandler, power_control::powerButtonLine, power_control::powerButtonEvent);

    // Request RESET_BUTTON GPIO events
    power_control::requestGPIOEvents("MON_RST_BTN_L", power_control::resetButtonHandler, power_control::resetButtonLine, power_control::resetButtonEvent);

    // Initialize the power state
    power_control::powerState = power_control::PowerState::off;
    // Check power good
    if (power_control::psPowerOKLine.get_value() > 0)
    {
        power_control::powerState = power_control::PowerState::on;
    }

    // Initialize the power state storage
    if (power_control::initializePowerStateStorage() < 0)
    {
        std::cerr << "initializePowerStateStorage return Error ";
        return -1;
    }

    power_control::chassisIntrusionMonitor();
    power_control::nmiSourcePropertyMonitor();

    std::cerr << "Initializing power state. ";
    power_control::logStateTransition(power_control::powerState);

    // Power Control Service
    sdbusplus::asio::object_server hostServer =
        sdbusplus::asio::object_server(power_control::conn);

    // Power Control Interface
    power_control::hostIface = hostServer.add_interface(
        "/xyz/openbmc_project/state/host0", "xyz.openbmc_project.State.Host");

    power_control::hostIface->register_property(
        "RequestedHostTransition",
        std::string("xyz.openbmc_project.State.Host.Transition.Off"),
        [](const std::string& requested, std::string& resp) {
            if (requested == "xyz.openbmc_project.State.Host.Transition.Off")
            {
                sendPowerControlEvent(
                    power_control::Event::gracefulPowerOffRequest);
                addRestartCause(power_control::RestartCause::command);
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Host.Transition.On")
            {
                sendPowerControlEvent(power_control::Event::powerOnRequest);
                addRestartCause(power_control::RestartCause::command);
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Host.Transition.Reboot")
            {
                sendPowerControlEvent(power_control::Event::powerCycleRequest);
                addRestartCause(power_control::RestartCause::command);
            }
            else if (requested == "xyz.openbmc_project.State.Host.Transition."
                                  "GracefulWarmReboot")
            {
                sendPowerControlEvent(
                    power_control::Event::gracefulPowerCycleRequest);
                addRestartCause(power_control::RestartCause::command);
            }
            else if (requested == "xyz.openbmc_project.State.Host.Transition."
                                  "ForceWarmReboot")
            {
                sendPowerControlEvent(power_control::Event::resetRequest);
                addRestartCause(power_control::RestartCause::command);
            }
            else
            {
                std::cerr << "Unrecognized host state transition request.\n";
                throw std::invalid_argument("Unrecognized Transition Request");
                return 0;
            }
            resp = requested;
            return 1;
        });
    power_control::hostIface->register_property(
        "CurrentHostState",
        std::string(power_control::getHostState(power_control::powerState)));

    power_control::hostIface->initialize();

    // Chassis Control Service
    sdbusplus::asio::object_server chassisServer =
        sdbusplus::asio::object_server(power_control::conn);

    // Chassis Control Interface
    power_control::chassisIface =
        chassisServer.add_interface("/xyz/openbmc_project/state/chassis0",
                                    "xyz.openbmc_project.State.Chassis");

    power_control::chassisIface->register_property(
        "RequestedPowerTransition",
        std::string("xyz.openbmc_project.State.Chassis.Transition.Off"),
        [](const std::string& requested, std::string& resp) {
            if (requested == "xyz.openbmc_project.State.Chassis.Transition.Off")
            {
                sendPowerControlEvent(power_control::Event::powerOffRequest);
                addRestartCause(power_control::RestartCause::command);
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Chassis.Transition.On")
            {
                sendPowerControlEvent(power_control::Event::powerOnRequest);
                addRestartCause(power_control::RestartCause::command);
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Chassis.Transition.PowerCycle")
            {
                sendPowerControlEvent(power_control::Event::powerCycleRequest);
                addRestartCause(power_control::RestartCause::command);
            }
            else
            {
                std::cerr << "Unrecognized chassis state transition request.\n";
                throw std::invalid_argument("Unrecognized Transition Request");
                return 0;
            }
            resp = requested;
            return 1;
        });
    power_control::chassisIface->register_property(
        "CurrentPowerState",
        std::string(power_control::getChassisState(power_control::powerState)));
    power_control::chassisIface->register_property(
        "LastStateChangeTime", power_control::getCurrentTimeMs());

    power_control::chassisIface->initialize();

    // Chassis System Service
    sdbusplus::asio::object_server chassisSysServer =
        sdbusplus::asio::object_server(power_control::conn);

    // Chassis System Interface
    power_control::chassisSysIface = chassisSysServer.add_interface(
        "/xyz/openbmc_project/state/chassis_system0",
        "xyz.openbmc_project.State.Chassis");

    power_control::chassisSysIface->register_property(
        "RequestedPowerTransition",
        std::string("xyz.openbmc_project.State.Chassis.Transition.On"),
        [](const std::string& requested, std::string& resp) {
            if (requested ==
                "xyz.openbmc_project.State.Chassis.Transition.PowerCycle")
            {
                power_control::systemReset();
                addRestartCause(power_control::RestartCause::command);
            }
            else
            {
                std::cerr << "Unrecognized chassis system state transition "
                             "request.\n";
                throw std::invalid_argument("Unrecognized Transition Request");
                return 0;
            }
            resp = requested;
            return 1;
        });
    power_control::chassisSysIface->register_property(
        "CurrentPowerState",
        std::string(power_control::getChassisState(power_control::powerState)));
    power_control::chassisSysIface->register_property(
        "LastStateChangeTime", power_control::getCurrentTimeMs());

    power_control::chassisSysIface->initialize();

    // Buttons Service
    sdbusplus::asio::object_server buttonsServer =
        sdbusplus::asio::object_server(power_control::conn);

    // Power Button Interface
    power_control::powerButtonIface = buttonsServer.add_interface(
        "/xyz/openbmc_project/chassis/buttons/power",
        "xyz.openbmc_project.Chassis.Buttons");

    // Check power button state
    bool powerButtonPressed = power_control::powerButtonLine.get_value() == 0;
    power_control::powerButtonIface->register_property("ButtonPressed",
                                                       powerButtonPressed);

    power_control::powerButtonIface->initialize();

    // Reset Button Interface
    power_control::resetButtonIface = buttonsServer.add_interface(
        "/xyz/openbmc_project/chassis/buttons/reset",
        "xyz.openbmc_project.Chassis.Buttons");

    // Check reset button state
    bool resetButtonPressed = power_control::resetButtonLine.get_value() == 0;
    std::cerr << "resetButtonPressed: "<< resetButtonPressed <<"!\n";
    power_control::resetButtonIface->register_property("ButtonPressed",
                                                       resetButtonPressed);

    power_control::resetButtonIface->initialize();


    // NMI Button Interface
    power_control::nmiButtonIface = buttonsServer.add_interface(
        "/xyz/openbmc_project/chassis/buttons/nmi",
        "xyz.openbmc_project.Chassis.Buttons");

    // Check NMI button state
    bool nmiButtonPressed = false;
    power_control::nmiButtonIface->register_property("ButtonPressed",
                                                      nmiButtonPressed);

    power_control::nmiButtonIface->initialize();

    // NMI out Service
    sdbusplus::asio::object_server nmiOutServer =
    sdbusplus::asio::object_server(power_control::conn);

    // NMI out Interface
    power_control::nmiOutIface =
    nmiOutServer.add_interface("/xyz/openbmc_project/control/host0/nmi",
                                       "xyz.openbmc_project.Control.Host.NMI");
    power_control::nmiOutIface->register_method("NMI",
                                                 power_control::nmiReset);
    power_control::nmiOutIface->initialize();


    // OS State Service
    sdbusplus::asio::object_server osServer =
        sdbusplus::asio::object_server(power_control::conn);

    // OS State Interface
    power_control::osIface = osServer.add_interface(
        "/xyz/openbmc_project/state/os",
        "xyz.openbmc_project.State.OperatingSystem.Status");

    // Get the initial OS state based on POST complete
    //      0: Asserted, OS state is "Standby" (ready to boot)
    //      1: De-Asserted, OS state is "Inactive"
    std::string osState = power_control::getGPIOValue("MON_POST_COMPLETE") > 0
                              ? "Standby"
                              : "Inactive";

    power_control::osIface->register_property("OperatingSystemState",
                                              std::string(osState));

    power_control::osIface->initialize();

    // Restart Cause Service
    sdbusplus::asio::object_server restartCauseServer =
        sdbusplus::asio::object_server(power_control::conn);

    // Restart Cause Interface
    power_control::restartCauseIface = restartCauseServer.add_interface(
        "/xyz/openbmc_project/control/host0/restart_cause",
        "xyz.openbmc_project.Control.Host.RestartCause");

    power_control::restartCauseIface->register_property(
        "RestartCause",
        std::string("xyz.openbmc_project.State.Host.RestartCause.Unknown"));

    power_control::restartCauseIface->register_property(
        "RequestedRestartCause",
        std::string("xyz.openbmc_project.State.Host.RestartCause.Unknown"),
        [](const std::string& requested, std::string& resp) {
            if (requested ==
                "xyz.openbmc_project.State.Host.RestartCause.WatchdogTimer")
            {
                power_control::addRestartCause(
                    power_control::RestartCause::watchdog);
            }
            else
            {
                throw std::invalid_argument(
                    "Unrecognized RestartCause Request");
                return 0;
            }

            std::cerr << "RestartCause requested: " << requested << "\n";
            resp = requested;
            return 1;
        });

    power_control::restartCauseIface->initialize();

    power_control::currentHostStateMonitor();

    gpiod::line gpioLine;
    // Set ASSERT_NMI_BTN_L to High
    if (!power_control::setGPIOOutput("ASSERT_NMI_BTN_L", 1, gpioLine))
    {
        return -1;
    }

    // Set ASSERT_RST_BTN_L to High
    if (!power_control::setGPIOOutput("ASSERT_RST_BTN_L", 1, gpioLine))
    {
        return -1;
    }

    // Set ASSERT_PWR_BTN_L to High
    if (!power_control::setGPIOOutput("ASSERT_PWR_BTN_L", 1, gpioLine))
    {
        return -1;
    }

    // Set BMC_READY to High
    if (!power_control::setGPIOOutput("ASSERT_BMC_READY", 1, gpioLine))
    {
        return -1;
    }

    // Check if we need to start the Power Restore policy
    power_control::powerRestorePolicyCheck();

    power_control::io.run();

    return 0;
}
