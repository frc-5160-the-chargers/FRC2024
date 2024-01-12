@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.commands

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.chargers.hardware.subsystems.differentialdrive.EncoderDifferentialDrivetrain
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.external.frc6328.characterization.FeedForwardCharacterization
import frc.external.frc6328.characterization.FeedForwardCharacterizationData
import frc.chargers.commands.commandbuilder.CommandBuilder
import frc.chargers.commands.commandbuilder.buildCommand

public fun characterizeFFAngular(
    name: String,
    forwards: Boolean = true,
    setVoltage: (Voltage) -> Unit,
    getVelocity: () -> AngularVelocity,
    vararg requirements: Subsystem
): FeedForwardCharacterization = object: FeedForwardCharacterization(
    forwards,
    FeedForwardCharacterizationData(name), { inputVolts -> setVoltage(inputVolts.ofUnit(volts))},
    {getVelocity().siValue},
    *requirements
){

    override fun initialize(){
        println("ANGULAR characterization is starting")
        super.initialize()
    }

    override fun end(interrupted: Boolean) {
        super.end(interrupted)
        println("Voltage UNIT: VOLTS")
        println("Angle UNIT: radians")
        println("Time UNIT: seconds")
    }
}

public fun characterizeFFLinear(
    name: String,
    forwards: Boolean,
    setVoltage: (Voltage) -> Unit,
    getVelocity: () -> Velocity,
    vararg requirements: Subsystem
): FeedForwardCharacterization = object: FeedForwardCharacterization(
    forwards,
    FeedForwardCharacterizationData(name), { inputVolts -> setVoltage(inputVolts.ofUnit(volts))},
    {getVelocity().siValue},
    *requirements
){

    override fun initialize(){
        println("LINEAR characterization is starting")
        super.initialize()
    }

    override fun end(interrupted: Boolean) {
        super.end(interrupted)
        println("Voltage UNIT: VOLTS")
        println("Angle UNIT: METERS")
        println("Time UNIT: SECONDS")
    }
}

context(CommandBuilder)
public fun EncoderHolonomicDrivetrain.characterizeDriveMotors(
    forwards: Boolean = true, vararg requirements: Subsystem
): FeedForwardCharacterization =
        object: FeedForwardCharacterization(
            forwards,
            FeedForwardCharacterizationData("SwerveDriveFFData_Left"),
            FeedForwardCharacterizationData("SwerveDriveFFData_Right"),
            {leftVolts: Double, rightVolts: Double ->
                topLeft.driveVoltage = leftVolts.ofUnit(volts)
                bottomLeft.driveVoltage = leftVolts.ofUnit(volts)
                topRight.driveVoltage = rightVolts.ofUnit(volts)
                bottomRight.driveVoltage = rightVolts.ofUnit(volts)

                topLeft.setDirection(0.degrees)
                topRight.setDirection(0.degrees)
                bottomLeft.setDirection(0.degrees)
                bottomRight.setDirection(0.degrees)
            },
            { (topLeft.speed + bottomLeft.speed).siValue / 2.0 },
            { (topRight.speed + bottomRight.speed).siValue / 2.0 },
            this, *requirements
        ){

            override fun initialize(){
                println("ANGULAR DRIVETRAIN characterization is starting")
                println("To stop the characterization, the command must be manually stopped!")
                super.initialize()
            }


            override fun end(interrupted: Boolean) {
                super.end(interrupted)
                println("Voltage UNIT: VOLTS")
                println("Angle UNIT: RADIANS")
                println("Time UNIT: SECONDS")
            }
        }.also(::addCommand)

context(CommandBuilder)
public fun EncoderHolonomicDrivetrain.characterizeTurnMotors(vararg requirements: Subsystem): Command =
    buildCommand{
        runParallelUntilAllFinish{
            +characterizeFFAngular(
                "TOP LEFT turn motor data",
                true,
                {topLeft.turnVoltage = it; println(it)},
                {topLeft.turnSpeed}
            )

            +characterizeFFAngular(
                "TOP RIGHT turn motor data",
                true,
                {topRight.turnVoltage = it},
                {topRight.turnSpeed}
            )

            +characterizeFFAngular(
                "BOTTOM LEFT turn motor data",
                true,
                {bottomLeft.turnVoltage = it},
                {bottomLeft.turnSpeed}
            )

            +characterizeFFAngular(
                "BOTTOM RIGHT turn motor data",
                true,
                {bottomRight.turnVoltage = it},
                {bottomRight.turnSpeed}
            )
        }
    }.withExtraRequirements(this@characterizeTurnMotors, *requirements).also(::addCommand)


context(CommandBuilder)
public fun EncoderDifferentialDrivetrain.characterize(
    forwards: Boolean = true, vararg requirements: Subsystem
): FeedForwardCharacterization =
    object: FeedForwardCharacterization(
        forwards,
        FeedForwardCharacterizationData("SwerveDriveFFData_Left"),
        FeedForwardCharacterizationData("SwerveDriveFFData_Right"),
        {leftVolts: Double, rightVolts -> setVoltages(leftVolts.volts, rightVolts.volts)},
        {leftVelocity.siValue},
        {rightVelocity.siValue},
        this, *requirements
    ){
        override fun initialize(){
            println("ANGULAR DRIVETRAIN characterization is starting.")
            println("To stop the characterization, the command must be manually stopped!")
            super.initialize()
        }

        override fun end(interrupted: Boolean) {
            super.end(interrupted)
            println("Voltage UNIT: VOLTS")
            println("Angle UNIT: RADIANS")
            println("Time UNIT: SECONDS")
        }
    }.also(::addCommand)

