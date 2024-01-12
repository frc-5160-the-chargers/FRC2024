@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.subsystems.differentialdrive.lowlevel

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.inches
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.framework.ChargerRobot

public class DiffDriveIOSim(
    logInputs: LoggableInputsProvider,
    motors: DifferentialDrivetrainSim.KitbotMotor
): DiffDriveIO {

    private val sim: DifferentialDrivetrainSim = DifferentialDrivetrainSim.createKitbotSim(
        motors,
        DifferentialDrivetrainSim.KitbotGearing.k10p71,
        DifferentialDrivetrainSim.KitbotWheelSize.kSixInch,
        null
    ).also{
        ChargerRobot.runPeriodically(addToFront = true){
            it.update(ChargerRobot.LOOP_PERIOD.inUnit(seconds))
        }
    }

    private var leftAppliedVoltage = Voltage(0.0)
    private var rightAppliedVoltage = Voltage(0.0)
    // gearRatio * wheelDiameter for simulator
    private val wheelTravelPerMotorRadian = 10.71 * 6.inches


    override fun setVoltages(left: Voltage, right: Voltage) {
        if (inverted){
            leftAppliedVoltage = left.coerceIn(-12.volts,12.volts)
            rightAppliedVoltage = right.coerceIn(-12.volts,12.volts)
        }else{
            leftAppliedVoltage = -left.coerceIn(-12.volts,12.volts)
            rightAppliedVoltage = -right.coerceIn(-12.volts,12.volts)
        }
        sim.setInputs(
            leftAppliedVoltage.inUnit(volts),
            rightAppliedVoltage.inUnit(volts)
        )
    }


    override val leftWheelTravel: Angle by logInputs.quantity{ sim.leftPositionMeters.ofUnit(meters) / wheelTravelPerMotorRadian }
    override val rightWheelTravel: Angle by logInputs.quantity{ sim.rightPositionMeters.ofUnit(meters) / wheelTravelPerMotorRadian }

    override val leftVelocity: AngularVelocity by logInputs.quantity{ sim.leftVelocityMetersPerSecond.ofUnit(meters / seconds) / wheelTravelPerMotorRadian }
    override val rightVelocity: AngularVelocity by logInputs.quantity{ sim.rightVelocityMetersPerSecond.ofUnit(meters / seconds) / wheelTravelPerMotorRadian }

    override val leftVoltage: Voltage by logInputs.quantity{leftAppliedVoltage}
    override val rightVoltage: Voltage by logInputs.quantity{rightAppliedVoltage}

    override var inverted: Boolean = false

}