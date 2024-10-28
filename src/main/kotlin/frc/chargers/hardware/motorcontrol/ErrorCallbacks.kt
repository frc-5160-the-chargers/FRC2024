package frc.chargers.hardware.motorcontrol

import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType

private val positionPIDAlert = Alert("Position PID must be configured via motor.configure(positionPID = PIDConstants(p,i,d))", AlertType.kError)
private val velocityPIDAlert = Alert("Velocity PID must be configured via motor.configure(velocityPID = PIDConstants(p,i,d))", AlertType.kError)

fun alertPositionPIDErr() {
    positionPIDAlert.set(true)
}

fun alertVelocityPIDErr() {
    velocityPIDAlert.set(true)
}