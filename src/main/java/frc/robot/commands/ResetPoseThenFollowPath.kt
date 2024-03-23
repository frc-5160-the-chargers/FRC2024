package frc.robot.commands

import com.batterystaple.kmeasure.units.meters
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.runOnceCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.utils.flipWhenNeeded
import frc.chargers.wpilibextensions.geometry.ofUnit

fun resetPoseThenFollowPath(
    drivetrain: EncoderHolonomicDrivetrain,
    path: PathPlannerPath
): Command = runOnceCommand{
    drivetrain.poseEstimator.resetPose(path.previewStartingHolonomicPose.ofUnit(meters).flipWhenNeeded())
}.andThen(
    AutoBuilder.followPath(path)
)