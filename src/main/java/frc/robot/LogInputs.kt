package frc.robot

import edu.wpi.first.wpilibj.RobotBase
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.robot.constants.ODOMETRY_LOCK

// the following use chargerlib's custom advantagekit wrapper: LoggableInputsProvider.
// These utilize property delegates to provide auto-logged inputs, I.E:
// val logInputs = LoggableInputsProvider("Hello")
// val property by logInputs.quantity{Angle(0.0)}
// val property2 by logInputs.double{0.0}
// these inputs will simply update and process themselves.

@JvmField
val OdometryLog = LoggableInputsProvider(
    "PoseData",
    updateInputs = RobotBase.isReal(),
    runBeforeInputUpdate = ODOMETRY_LOCK::lock,
    runAfterInputUpdate = ODOMETRY_LOCK::unlock
)