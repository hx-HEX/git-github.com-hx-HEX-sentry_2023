#pragma once

#include "system_monitor.h"
#include "led.h"
#include "vofa.h"
#include "gimbal.h"
#include "bmi088.h"
#include "sentry_robot.h"
#include "can_interface.h"

#define GIMBAL_DEBUG

class Global {
public:
    // instance
    static SystemMonitor system_monitor;
    static Led led;
    static Vofa vofa;
    static Gimbal gimbal;
    static SentryRobot sentry;

    // states
    enum ControlMode {
        SAFE = 0,
        CALIBRATE,
        RC_GS,
        AUTO_G_RC_S,
        AUTO_GS_RC_C,
        AUTO_CGS,
        RC_C,
        AUTO_G_RC_C
    };
    static ControlMode control_mode; 

    static void ControlModeUpdate(void);
    static void RobotStatesUpdate(void);
    static void RobotTargetsUpdate(void);

    static void VisualizeGimbalData(void);
    static void RobotControlExecute(void);

};







