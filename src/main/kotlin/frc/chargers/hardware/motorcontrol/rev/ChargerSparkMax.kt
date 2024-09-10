package frc.chargers.hardware.motorcontrol.rev

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkLowLevel


/**
 * Creates a [CANSparkMax] that implements the [frc.chargers.hardware.motorcontrol.Motor] and
 * [frc.chargers.framework.faultchecking.FaultChecking] interfaces.
 *
 * @see ChargerSpark
 */
class ChargerSparkMax(
    deviceID: Int,
    motorType: CANSparkLowLevel.MotorType = CANSparkLowLevel.MotorType.kBrushless,
    useAbsoluteEncoder: Boolean = false,
    factoryDefault: Boolean = true
): ChargerSpark<CANSparkMax>(CANSparkMax(deviceID, motorType), useAbsoluteEncoder, factoryDefault)