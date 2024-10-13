package frc.chargers.hardware.motorcontrol

import frc.chargers.framework.HorseLog.logError

fun alertPositionPIDErr() {
    logError(
        "Position PID not configured",
        "You must specify position PID constants via motor.configure(positionPID = PIDConstants(p,i,d))."
    )
}

fun alertVelocityPIDErr() {
    logError(
        "Velocity PID not configured",
        "You must specify velocity PID constants via motor.configure(velocityPID = PIDConstants(p,i,d))."
    )
}