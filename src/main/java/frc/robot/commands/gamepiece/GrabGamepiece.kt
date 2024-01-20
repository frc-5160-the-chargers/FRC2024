package frc.robot.commands.gamepiece

import com.batterystaple.kmeasure.quantities.Scalar
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.hardware.sensors.vision.MLVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.constants.DEFAULT_AIMING_PID
import frc.robot.hardware.subsystems.groundintake.GroundIntake
import frc.robot.hardware.subsystems.shooter.Shooter

private val GROUND_INTAKE_DEFAULT_VOLTAGE = 8.volts
private val SHOOTER_INTAKE_DEFAULT_VOLTAGE = 5.volts

private const val DEFAULT_DRIVE_POWER = 0.0


fun grabGamepiece(
    drivePower: Double = DEFAULT_DRIVE_POWER,
    aimingPID: PIDConstants = DEFAULT_AIMING_PID,

    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    groundIntake: GroundIntake,
    visionIO: MLVisionPipeline,
): Command = buildCommand {
    addRequirements(drivetrain, shooter, groundIntake)

    fun spinIntakesAndDrive(strafe: Double = 0.0){
        drivetrain.swerveDrive(drivePower, strafe, 0.0)
        groundIntake.spin(GROUND_INTAKE_DEFAULT_VOLTAGE)
        shooter.spin(SHOOTER_INTAKE_DEFAULT_VOLTAGE)
    }

    runOnce{
        visionIO.reset()
        visionIO.require()
    }

    val aimController by getOnceDuringRun{
        SuperPIDController(
            aimingPID,
            getInput = { Scalar(visionIO.bestTarget?.tx ?: 0.0) },
            target = Scalar(0.0),
            outputRange = Scalar(-0.5)..Scalar(0.5)
        )
    }

    if (shooter.canDetectGamepieces){
        loopUntil( { shooter.hasGamepiece } ){
            spinIntakesAndDrive(
                strafe = aimController.calculateOutput().siValue
            )
        }
    }else{
        loopFor(3.seconds){
            spinIntakesAndDrive(
                strafe = aimController.calculateOutput().siValue
            )
        }
    }

    runOnce{
        visionIO.removeRequirement()
        drivetrain.stop()
        shooter.setPivotVoltage(0.volts)
        shooter.spin(0.0)
    }
}