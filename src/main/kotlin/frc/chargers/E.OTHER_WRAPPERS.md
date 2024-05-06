# Hardware Wrappers

Chargerlib as a whole is a library that is built on the idea of abstraction between hardware layers.

A variety of vendors already provide hardware-interface classes for specific pieces of hardware; like the ```CANSparkMax()```, ```CANcoder()``` and ```PhotonCamera()``` classes. 
However, the methods used to retreive data and call actions on these pieces of hardware usually vary from vendor-to-vendor; this makes it hard to switch between components. 
For instance, if you had an Arm Subsystem that started off with a CANSparkMax, it would be very hard to switch that motor to a TalonFX.

In addition, most vendor classes don't support kmeasure units(due to it being a custom library),
which means that without wrappers, we will often have to do manual units conversions as well.

ChargerLib solves this problem with wrapper classes and interfaces, which abstract away most of the necessary implementation between classes,
and return Kmeasure Quantities instead of bare Doubles. 

If you just need a generic use-case motor without any vendor-specific features, you can now simply accept a generic ```EncoderMotorController``` instead.

Most wrapper classes start with the term "Charger" as a prefix; for instance, ```ChargerCANcoder()```, ```ChargerSparkMax()```, and ```ChargerPhotonCam()```.

Disclaimer: the subsystem implementations below are non-advantagekit. To see advantagekit implementations, check the AdvantageKit Wrapper section(tbd).



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


# Motor Controllers

WPILib already contains the ```MotorController``` interface, which abstracts away most motor controller functions, including setting power and voltage. However, this does not encompass the motor's built in encoder. Thus, ChargerLib has an ```EncoderMotorController``` interface, which adds a subclassed ```Encoder``` to the ```MotorController``` interface.

```kotlin
   public class Elevator(val motor: EncoderMotorController, ....){
       public val position: Distance get() = motor.encoder.angularPosition * 5.meters // * 5 meters transforms the Angle into a Distance
       public val velocity: Velocity get() = motor.encoder.angularVelocity * 5.meters // * 5 meters transforms the AngularVelocity into a Velocity
   }
   
   // neoSparkMax implements Encoder
   val elevatorInstance = Elevator(ChargerSparkMax(6), ....)
```

In addition, the 2024 version of ChargerLib includes the ```SmartEncoderMotorController``` interface,
which allows you to fetch the temperature(```motor.tempCelsius```; a Double because Kmeasure does not support temperature units), 
applied current(```motor.appliedCurrent```) and applied voltage(```motor.appliedVoltage```) of the motor,
and provides support for closed loop control via ```setAngularPosition``` and ```setAngularVelocity```.

Implementing classes of SmartEncoderMotorController:
```ChargerSparkMax(id)```, ```ChargerSparkFlex(id)```, ```ChargerTalonFX(id)```

Implementing classes of only EncoderMotorController:
```ChargerTalonSRX(id)```



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

# Pose Estimation


Robot pose estimators are represented using the ```RobotPoseMonitor``` abstract class.

There are a couple of o

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
