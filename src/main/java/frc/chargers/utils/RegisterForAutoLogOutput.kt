package frc.chargers.utils

import org.littletonrobotics.junction.AutoLogOutputManager

/**
 * Credits: Team 749
 * https://github.com/FRC-Team-955/2024-RobotCode-749/blob/kotlin/src/main/java/frc/robot/Util.kt
 */
fun registerSingletonsForAutoLogOutput(
    vararg roots: Any
){
    roots.forEach { root ->
        val method = AutoLogOutputManager::class.java.getDeclaredMethod("registerFields", Object::class.java)
        method.isAccessible = true
        method.invoke(null, root)
    }
}