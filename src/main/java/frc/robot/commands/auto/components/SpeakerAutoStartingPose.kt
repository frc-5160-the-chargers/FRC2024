@file:Suppress("unused")
package frc.robot.commands.auto.components

import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d

enum class SpeakerAutoStartingPose(val pose: UnitPose2d){
    LEFT(UnitPose2d()),
    CENTER(UnitPose2d()),
    RIGHT(UnitPose2d())
}