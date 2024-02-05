@file:Suppress("unused")
package frc.robot.commands

import com.batterystaple.kmeasure.quantities.Scalar
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.constants.OPEN_LOOP_TRANSLATION_PID
import frc.robot.hardware.subsystems.groundintake.GroundIntake
import frc.robot.hardware.subsystems.shooter.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

private val GROUND_INTAKE_DEFAULT_VOLTAGE = 8.volts
private val SHOOTER_INTAKE_DEFAULT_VOLTAGE = -5.volts

private const val DEFAULT_DRIVE_POWER = 0.0


fun grabGamepiece(
    drivePower: Double = DEFAULT_DRIVE_POWER,
    visionPipeline: ObjectVisionPipeline,

    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    groundIntake: GroundIntake,
): Command = buildCommand {
    addRequirements(drivetrain, shooter, groundIntake)

    +shooter.setAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF)

    runOnce{
        visionPipeline.requireAndReset()
    }

    val aimController by getOnceDuringRun{
        SuperPIDController(
            OPEN_LOOP_TRANSLATION_PID,
            getInput = { Scalar(visionPipeline.bestTarget?.tx ?: 0.0) },
            target = Scalar(0.0),
            outputRange = Scalar(-0.5)..Scalar(0.5)
        )
    }

    fun spinIntakesAndDrive(strafe: Double = 0.0){
        drivetrain.swerveDrive(drivePower, strafe, 0.0)
        groundIntake.spin(GROUND_INTAKE_DEFAULT_VOLTAGE)
        shooter.setVoltage(SHOOTER_INTAKE_DEFAULT_VOLTAGE)
    }

    fun shooterHasGamepiece() = shooter.hasGamepiece

    if (shooter.canDetectGamepieces){
        loopUntil(::shooterHasGamepiece){
            spinIntakesAndDrive(strafe = aimController.calculateOutput().siValue)
        }
    }else{
        loopFor(3.seconds){
            spinIntakesAndDrive(strafe = aimController.calculateOutput().siValue)
        }
    }

    runOnce{
        visionPipeline.removeRequirement()
        drivetrain.stop()
        groundIntake.spin(0.0)
        shooter.setPivotVoltage(0.volts)
        shooter.setSpeed(0.0)
    }
}