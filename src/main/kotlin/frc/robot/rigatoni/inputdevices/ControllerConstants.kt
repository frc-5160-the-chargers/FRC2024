package frc.robot.rigatoni.inputdevices

@Suppress("unused")
enum class Driver(val rightHanded: Boolean) {
    NAYAN(false),
    KENNA(true),
    CONRAD(true),
    JOYCE(true),
    JACK(true)
}

// controller Ports
const val DRIVER_CONTROLLER_PORT = 0
const val OPERATOR_CONTROLLER_PORT = 1

val DRIVER = Driver.NAYAN
const val DEFAULT_DEADBAND = 0.1
const val SHOULD_INVERT_SHOOTER_SPEED = true
const val SHOULD_INVERT_PIVOT_SPEED = true