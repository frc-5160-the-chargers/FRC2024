@file:Suppress("unused")
package frc.robot

// controller Ports
const val DRIVER_CONTROLLER_PORT = 0
const val OPERATOR_CONTROLLER_PORT = 1

// drivetrain IDs
// Top is Ground Intake side
object DrivetrainID{
    const val TL_DRIVE = 10
    const val TR_DRIVE = 3
    const val BL_DRIVE = 16
    const val BR_DRIVE = 30

    const val TL_TURN = 12
    const val TR_TURN = 4
    const val BL_TURN = 31
    const val BR_TURN = 22

    const val TL_ENCODER = 44
    const val TR_ENCODER = 45
    const val BL_ENCODER = 42
    const val BR_ENCODER = 43
}

// mechanism IDs
const val SHOOTER_MOTOR_ID = 7
const val PIVOT_MOTOR_ID = 9
const val GROUND_INTAKE_ID = 7
const val CONVEYOR_ID = 29
const val CLIMBER_ID_LEFT = 6
const val CLIMBER_ID_RIGHT = 8

// Sensor IDs
const val SHOOTER_SENSOR_ID = 9
const val GROUND_INTAKE_SENSOR_ID = 1
const val PIVOT_ENCODER_ID = 0