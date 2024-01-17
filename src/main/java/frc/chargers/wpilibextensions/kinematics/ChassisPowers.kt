// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.wpilibextensions.kinematics

import com.batterystaple.kmeasure.quantities.*
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.chargers.advantagekitextensions.AdvantageKitLoggable
import org.littletonrobotics.junction.LogTable
import kotlin.math.abs

/**
 * A helper class that stores direction powers for drivetrain classes.
 */
public data class ChassisPowers(
    var xPower: Double = 0.0,
    var yPower: Double = 0.0,
    var rotationPower: Double = 0.0
): AdvantageKitLoggable<ChassisPowers> {
    public fun toChassisSpeeds(
        maxLinearVelocity: Velocity,
        maxRotationalVelocity: AngularVelocity
    ): ChassisSpeeds = ChassisSpeeds(
        xPower * maxLinearVelocity,
        yPower * maxLinearVelocity,
        rotationPower * maxRotationalVelocity
    )

    /**
     * Measures whether 2 [ChassisPowers] are roughly equal.
     */
    public infix fun roughlyEquals(other: ChassisPowers): Boolean =
        abs(xPower - other.xPower) <= 0.01
            && abs(yPower - other.yPower) <= 0.01
            && abs(rotationPower - other.rotationPower) <= 0.01

    override fun pushToLog(table: LogTable, category: String) {
        table.put("$category/xPower", xPower)
        table.put("$category/yPower", yPower)
        table.put("$category/rotationPower", rotationPower)
    }

    override fun getFromLog(table: LogTable, category: String): ChassisPowers =
        ChassisPowers(
            table.get("$category/xPower", 0.0),
            table.get("$category/yPower", 0.0),
            table.get("$category/rotationPower", 0.0)
        )
}





