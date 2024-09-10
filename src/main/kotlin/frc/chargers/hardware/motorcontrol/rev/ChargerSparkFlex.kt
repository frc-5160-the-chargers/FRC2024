package frc.chargers.hardware.motorcontrol.rev

import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel


/**
 * Creates a [CANSparkFlex] that implements the [frc.chargers.hardware.motorcontrol.Motor]
 * and [frc.chargers.framework.faultchecking.FaultChecking] interfaces.
 *
 * @see ChargerSpark
 */
class ChargerSparkFlex(
    deviceID: Int,
    useAbsoluteEncoder: Boolean = false,
    factoryDefault: Boolean = true
): ChargerSpark<CANSparkFlex>(CANSparkFlex(deviceID, CANSparkLowLevel.MotorType.kBrushless), useAbsoluteEncoder, factoryDefault)