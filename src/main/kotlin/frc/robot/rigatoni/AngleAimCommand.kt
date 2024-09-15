package frc.robot.rigatoni

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.controls.PIDController
import frc.chargers.controls.motionprofiling.AngularMotionProfile
import frc.chargers.controls.motionprofiling.AngularMotionProfileState
import frc.chargers.controls.motionprofiling.trapezoidal.AngularTrapezoidProfile
import frc.chargers.framework.Loggable
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.wpilibextensions.flipWhenRedAlliance
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import kotlin.math.PI
import kotlin.math.atan2

class AngleAimCommand(
    private val getTarget: () -> Angle,
    private val drivetrain: EncoderHolonomicDrivetrain,
    private val getChassisPowers: () -> ChassisPowers,
): Command(), Loggable {
    constructor(target: Angle, drivetrain: EncoderHolonomicDrivetrain, getChassisPowers: () -> ChassisPowers):
        this({target}, drivetrain, getChassisPowers)

    constructor(
        targetPose: Pose2d,
        drivetrain: EncoderHolonomicDrivetrain,
        getChassisPowers: () -> ChassisPowers,
        angleOffset: Angle = Angle(0.0)
    ): this(
        {
            val drivetrainToTagTranslation = (targetPose.translation - drivetrain.robotPose.translation)
            atan2(drivetrainToTagTranslation.y, drivetrainToTagTranslation.x).ofUnit(radians) + angleOffset
        },
        drivetrain,
        getChassisPowers
    )

    override val namespace = "AimToAngleController"
    private val pidController = PIDController(AIM_TO_ANGLE_PID)
    private val motionProfile: AngularMotionProfile = AngularTrapezoidProfile(
        drivetrain.maxRotationalVelocity,
        drivetrain.maxRotationalVelocity / 1.5.seconds
    )
    private var setpoint = AngularMotionProfileState(drivetrain.heading)
    private var goal = AngularMotionProfileState()

    init {
        pidController.enableContinuousInput(0.0, 2 * PI)
        addRequirements(drivetrain)
    }

    override fun execute() {
        goal.position = getTarget().flipWhenRedAlliance()
        setpoint = motionProfile.calculate(setpoint, goal)
        val swerveOutput = getChassisPowers()
        swerveOutput.rotationPower = pidController.calculate(drivetrain.heading.siValue, setpoint.position.siValue)
        drivetrain.swerveDrive(swerveOutput, fieldRelative = true)
    }
}