# Hardware Wrappers

Chargerlib as a whole is a library that is built on the idea of abstraction between hardware layers.

A variety of vendors already provide hardware-interface classes for specific pieces of hardware; like the ```CANSparkMax()```, ```CANcoder()``` and ```TalonFX()``` classes. 
However, the methods used to retreive data and call actions on these pieces of hardware usually vary from vendor-to-vendor; this makes it hard to switch between components. 
For instance, if you had an Arm Subsystem that started off with a CANSparkMax, it would be very hard to switch that motor to a TalonFX or a simulated motor.

In addition, most vendor classes don't support kmeasure units(due to it being a custom library),
which means that without wrappers, we will often have to do manual units conversions as well.

ChargerLib solves this problem with wrapper classes and interfaces, which abstract away most of the necessary implementation between classes,
and return Kmeasure Quantities instead of bare Doubles. 

If you just need a generic use-case motor without any vendor-specific features, you can now simply accept a generic ```EncoderMotorController``` instead.

Most wrapper classes start with the term "Charger" as a prefix; for instance, ```ChargerCANcoder()```, ```ChargerSparkMax()```, and ```ChargerTalonFX()```.


# Encoders

Encoders within chargerlib are handled via 2 interfaces:

1. ```PositionEncoder```(which returns the measured angular position), and
2. ```VelocityEncoder```(which returns the measured angular velocity).

For most use cases, you should simply use the ```Encoder``` interface, which extends both ```PositionEncoder``` and ```VelocityEncoder```.

```kotlin
   public class SingleJointedArm(val encoder: Encoder, ....){
       public val position: Angle get() = encoder.angularPosition
       public val velocity: AngularVelocity get() = encoder.angularPosition
   }
   
   // ChargerCANcoder implements Encoder
   val armInstance = SingleJointedArm(ChargerCANcoder(5), ....)
``` 

Encoder implementing classes: ```ChargerCANcoder(id)``(provides timestamps)` and ```ChargerQuadEncoder(port)```(WPILib encoder wrapper)
PositionEncoder implementing classes: ```ChargerPotentiometer(port)``` and ```ChargerDutyCycleEncoder(id)```


# Motor Controlling

We utilize the ```MotorizedComponent``` interface to represent 1 or more motors, along with a respective encoder, 
that controls 1 component on the robot. This can include spinning a set of wheels on a flywheel, or spinning 1 side of an intake.

In order to set the voltage of the motor and get the voltage of the motor, you would use the ```appliedVoltage``` getter-setter property.
A more convenient way to do this is with the ```percentOut``` property, which allows you to set the motor's output
as a percentage of 12 volts instead.

Here are the ways to retreive motor data:
    For encoder readings, call  ```MotorizedComponent.encoder```, which returns an ```Encoder```.
    For stator current readings, call ```MotorizedComponent.statorCurrent```, which returns a ```Quantity<CurrentDimension>```.
    For setting and fetching motor inversion, call the ```MotorizedComponent.hasInvert``` getter-setter property.

```kotlin
   public class Elevator(val motor: MotorizedComponent, ....){
   
       init{
            motor.hasInvert = true
            val current: Quantity<CurrentDimension> = motor.statorCurrent
            // Current is a typealias for Quantity<StatorCurrent>
            val currentAlt: Current = motor.statorCurrent
       }
      
       fun run(){
            motor.appliedVoltage = 5.volts
            //motor.percentOut = -0.3
       }
       
       public val position: Distance get() = motor.encoder.angularPosition * 5.meters // * 5 meters transforms the Angle into a Distance
       public val velocity: Velocity get() = motor.encoder.angularVelocity * 5.meters // * 5 meters transforms the AngularVelocity into a Velocity
   }
   
   // neoSparkMax implements Encoder
   val elevatorInstance = Elevator(ChargerSparkMax(6), ....)
   
   val simulatedElevatorInstance = Elevator(MotorSim(...)...)
```


Implementing classes of MotorizedComponent:
```ChargerSparkMax(id)```, ```ChargerSparkFlex(id)```, ```ChargerTalonFX(id)```, ```ChargerTalonSRX(id)```

```MotorSim(...)```, ```ArmMotorSim(...)```, ```ElevatorMotorSim(...)```



# Gyroscopes, HeadingProvider and IMU

A basic class that can return the heading of the robot is represented as a ```HeadingProvider```, 
which is accessed using the ```heading``` property:

```kotlin
// drivetrains can provide heading via encoder readings
val headingProvider: HeadingProvider = EncoderDifferentialDrivetrain(....)
val headingProviderTwo: HeadingProvider = ChargerNavX()

val heading: Angle get() = headingProviderTwo.heading
```

A heading provider that can be zeroed is represented using the  ```ZeroableHeadingProvider``` interface, 
which has the ```zeroHeading(angle: Angle)``` function. When called without a specified angle, 
this function will set the angle to 0 degrees.

In addition, there are the ```ThreeAxisGyroscope```, ```ThreeAxisAccelerometer``` and ```ThreeAxisSpeedometer``` classes, 
which measure angle, acceleration and velocity, respectively, in all 3 axes. For instance:

```kotlin
val navX = ChargerNavX()
val gyroscope: ThreeAxisGyroscope = navX.gyroscope

println(gyroscope.yaw + gyroscope.pitch + gyroscope.roll) // Angle

val accelerometer: ThreeAxisAccelerometer = navX.accelerometer

println(accelerometer.xAcceleration + accelerometer.yAcceleration + ...) // Acceleration

val speedometer: ThreeAxisSpeedometer = navX.speedometer

println(accelerometer.xVelocity + accelerometer.yVelocity + ...) // Velocity
```

These interfaces are usually subclassed properties of overarching IMU classes.


HeadingProvider implementing classes: drivetrain subsystems, ```ChargerPigeon2()```, ```ChargerNavX()```

```ChargerPigeon2(id)``` has subclassed ThreeAxisGyroscope and ThreeAxisAccelerometer implementations.
In addition, it supports fetching yaw rate, pitch rate and roll rate as well.

```ChargerNavX()```  has subclassed ThreeAxisGyroscope, ThreeAxisAccelerometer and ThreeAxisSpeedometer impls.


# Vision
Instead of providing abstractions per vision camera, ChargerLib provides the nessecary abstractions for every vision pipeline.

Fetched data from a vision camera is represented as the ```VisionTarget``` class.
At the moment, there are 2 different kinds of vision targets: 

A. ```VisionTarget.Object(timestamp: Time, tx: Double, ty: Double, areaPercent: Double, classId: String?)```,
which represents a generic object or gamepiece detected by a vision camera, regardless of implementation(color or ML pipelines).

B. ```VisionTarget.AprilTag(timestamp: Time, tx: Double, ty: Double, areaPercent: Double, fiducialId: Int, targetTransformFromCam: UnitTransform3d)```, 
which represents an AprilTag vision target. Note: UnitTransform3d is simply a Transform3d with units support.

Nessecary implementing methods and properties include:

A. ```val visionTargets: List<T: VisionTarget>```: represents the current vision results/targets obtained.

B. ```val cameraConstants: VisionCameraConstants```: holds constants related to the vision camera.

C. ```fun reset()```: resets the vision pipeline. 
                    This usually involves resetting the pipeline index of the overarching vision camera, 
                    but it can do other things. 
                    Recommended to call at the initialize() block of commands and the init{} block of subsystems(the constructor).

Implemented methods:

A. ```val bestTarget: R?```: Simply fetches the best result from the visionData getter, returning null if no targets are present.

A. ```fun distanceToTarget(target: VisionTarget)```: calculates the horizontal distance to target using the lens height and mount angle.

B. ```fun diagonalDistanceToTarget(target: VisionTarget)```: calculates the diagonal distance to target using the lens height and mount angle.

Example usage:

```kotlin
val ll = ChargerLimelight(LENS_HEIGHT, MOUNT_ANGLE)
var apriltagPipeline: VisionPipeline<VisionResult.AprilTag> = ll.ApriltagPipeline(6, ....)

apriltagPipeline.reset()

println(apriltagPipeline.bestTarget?.tx)
println(apriltagPipeline.visionTargets[1].targetTransformFromCam)

var apriltagPipeline = ChargerPhotonCam("5160 photon cam", ....).ApriltagPipeline(5, ....)
```

# Pose Estimation

Vision cameras that can provide pose are represented using the  ```VisionPoseSupplier``` interface.
This interface has a singular property: ```robotPose```, which returns a ```UnitPose2d```(a pose with units).
In addition, the interface must provide the camera yaw as well, 
in order to calculate proper standard deviations.

On the other hand, robot pose estimators are represented using the ```RobotPoseMonitor``` interface.
This interface's ```robotPose``` property is not nullable. 

More information:

1. To reset the pose of the estimator, call ```resetPose(UnitPose2d)``` and the ```resetPose()``` functions.
2. To add vision pose suppliers, call the ```addPoseSuppliers(vararg visionSystems: VisionPoseSupplier)``` method.


# Configuration of Motors, Encoders, and other hardware

Instead of being configured within the subsystem, ChargerLib hardware classes that are configurable all have the option to configure themselves BEFORE they are passed into the subsystem. 
This allows for the use of other abstract interfaces that don't necessarily provide configuration.

Configuration in ChargerLib is abstracted using the ```HardwareConfiguration``` and ```HardwareConfigurable<HardwareConfiguration>``` interfaces.

All configuration objects implement the ```HardwareConfiguration``` interface, which has no required implementation of it's own:

```kotlin
class ChargerSparkMaxConfiguration(....): HardwareConfiguration
```

While configurable hardware implements the ```HardwareConfigurable<C: HardwareConfiguration>``` interface:

```kotlin
class ChargerSparkMax(....): HardwareConfigurable<ChargerSparkMaxConfiguration>{
    override fun configure(configuration: ChargerSparkMaxConfiguration){....}
}
```
All HardwareConfigurable's must implement the ```configure``` function, which configures the class.

While not enforced, it is standard for HardwareConfiguration's to have only nullable input parameters, with null representing an unconfigured parameter. This allows for configuration multiple times without overriding existing configs.

It is usually customary for ```MotorConfigurable```s to have inline factory functions which accept a "configure" function in the end that is applied to a new configuration before configuring the motor. For instance:

```kotlin
var sparkMax = ChargerSparkMax(6, configuration = ChargerSparkMaxConfiguration(...))

// factory defaulting on initialization is optional


val sparkFlex = ChargerSparkFlex(7){
    // this block has the context of a ChargerSparkMaxConfiguration(essentially equivalent to calling the function WITHIN the config class itself).
    inverted = false
    voltageCompensationNominalVoltage = 12.volts
    ....
}

val talonFX = ChargerTalonFX(6){
    // this block has the context of a ChargerTalonFXConfiguration(essentially equivalent to calling the function WITHIN the config class itself).
    forwardSoftLimitEnable = true
    forwardSoftLimitTreshold = Angle(5.0)
}

```
