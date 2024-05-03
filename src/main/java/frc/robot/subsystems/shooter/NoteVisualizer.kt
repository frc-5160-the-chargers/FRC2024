@file:Suppress("unused")
package frc.robot.subsystems.shooter

import com.batterystaple.kmeasure.quantities.Voltage
import edu.wpi.first.math.geometry.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ScheduleCommand
import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.Loggable

// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
// Note: This code is mostly identical to mechanical advantage's code,
// aside from being converted to kotlin.

object NoteVisualizer: Loggable {
    override val namespace = "Shooter"

    private val blueSpeaker = Translation3d(0.225, 5.55, 2.1)
    private val redSpeaker = Translation3d(16.317, 5.55, 2.1)
    private val zeroedNoteOffset = Transform3d(
        Translation3d(0.0,-0.03, 0.15),
        Rotation3d()
    )

    private const val shotSpeed = 5.0 // Meters per sec
    private var robotPoseSupplier = { Pose2d() }
    private var launcherTransformSupplier = { Transform3d() }

    private var hasNoteInShooter: Boolean = false


    var isShootingInSpeaker: Boolean = false
        private set

    init{
        ChargerRobot.runPeriodicLowPriority {
            if (hasNoteInShooter){
                log(
                    Pose3d.struct,
                    "Shooter/notePose",
                    listOf(
                        Pose3d(robotPoseSupplier())
                            .transformBy(launcherTransformSupplier())
                            .transformBy(zeroedNoteOffset)
                    )
                )
            }else if (!isShootingInSpeaker){
                log(Pose3d.struct, "Shooter/notePose", listOf())
            }
        }
    }

    fun setRobotPoseSupplier(supplier: () -> Pose2d){
        robotPoseSupplier = supplier
    }

    // regular pose3d as that is whats being logged
    fun setLauncherTransformSupplier(supplier: () -> Pose3d){
        launcherTransformSupplier = { supplier() - Pose3d() }
    }

    fun shootInSpeakerCommand(): Command {
        if (RobotBase.isReal()) return InstantCommand()
        return ScheduleCommand( // Branch off and exit immediately
            Commands.defer(
                {
                    val startPose =
                        Pose3d(robotPoseSupplier()).transformBy(launcherTransformSupplier())
                    val isRed =
                        DriverStation.getAlliance().isPresent && DriverStation.getAlliance().get() == Alliance.Red
                    val endPose =
                        Pose3d(if (isRed) redSpeaker else blueSpeaker, startPose.rotation)

                    val duration =
                        startPose.translation.getDistance(endPose.translation) / shotSpeed
                    val timer = Timer()
                    timer.start()
                    Commands.run(
                        {
                            isShootingInSpeaker = true
                            log(
                                Pose3d.struct,
                                "Shooter/notePose",
                                listOf(
                                    startPose
                                        .interpolate(endPose, timer.get() / duration)
                                        .transformBy(zeroedNoteOffset)
                                )
                            )
                        })
                        .until { timer.hasElapsed(duration) }
                        .finallyDo { _: Boolean ->
                            log(Pose3d.struct, "Shooter/notePose", listOf())
                            isShootingInSpeaker = false
                        }
                },
                setOf()
            ).ignoringDisable(true)
        )
    }


    fun update(shooterVoltage: Voltage){
        if (RobotBase.isReal()) return

        if (shooterVoltage.siValue < 0.0) {
            hasNoteInShooter = true
        }else if (shooterVoltage.siValue > 0.0){
            hasNoteInShooter = false
        }
    }

    fun setHasNote(hasNote: Boolean){
        if (RobotBase.isReal()) return
        hasNoteInShooter = hasNote
    }
}