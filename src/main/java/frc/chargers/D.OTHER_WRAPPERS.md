# Other Units Wrappers

## PID control

ChargerLib introduces the ```SuperPIDController``` class that adds many improvements onto the base PIDController and ProfiledPIDController classes.

Here are the listed improvements:
1. Automatic sustaining: SuperPIDController automatically calls itself every loop, which removes the risk of the controller breaking if it's output is not constantly being used.
2. Units support: SuperPIDController accepts a Quantity as an input, converts it to it's siValue, applies the PID algorithm, then converts it to the appropriate output unit. This ensures that the PID constants being tuned are always applicable for the specific mechanism.
3. Built-in feedforward and motion profiling(trapezoidal and exponential profiles!)

Here is a basic diagram of how it works:




Essentially, the controller target, or goal, is passed into a ```SetpointSupplier<S: AnyDimension, O: AnyDimension>```, which produces a ```Setpoint<S,O>``` class through the getSetpoint function. This class contains the setpoint value, as well as the feedforward output. From there, the setpoint is passed into a regular PID control algorithm, while the feedforward output is added at the end to return the final output.

### Feedforward control

Feedforward control is handled through the ```Feedforward<I: AnyDimension, O: AnyDimension>``` functional interface.
This has 1 method: calculate, which takes a target of type Quantity<I> and outputs a control effort of type Quantity<O>.

Because Feedforward is a functional interface, custom feedforwards can be constructed using SAM notation:

```kotlin
val ff = Feedforward{ velocity: Velocity -> somePolynomialFunction(velocity) }
```

However, the most common use case of Feedforwards is using characterized gains in order to get an equation. T
his hinges on the various FFConstants classes, which include:

```AngularMotorFFConstants(kS, kV, kA)``` - Holds constants for a simple motor feedforward, with angular velocity,
```LinearMotorFFConstants(kS, kV, kA)``` - Holds constants for a simple motor feedforward, with linear velocity,

```ArmFFConstants(kS, kG, kV, kA)```- Holds constants for an arm motor feedforward, with angular velocity,
```ElevatorFFConstants(kS, kG, kV, kA)``` - Holds constants for an elevator feedforward, with linear velocity.

These classes use typesafe units for templating; however, they all provide a ```fromSI``` factory function
that takes Doubles(which correspond to the SI values) instead. 

For angular feedforwards, these are radians, seconds, and volts, 
while in linear feedforwards, these are meters, seconds, and volts.

In order to utilize these feedforward constants, simply construct a Feedforward like so:

```kotlin

var angularConsts = AngularMotorFFConstants( 0.1.volts, 0.2.volts / (1.meters / 1.seconds), 0.1.volts / (1.meters / 1.seconds / 1.seconds) )
angularConsts = AngularMotorFFConstants.fromSI(0.1, 0.2, 0.1)

// Feedforward is also a factory function here
val angularMotorFF = Feedforward(angularConsts, getTargetAccel = { Acceleration(0.0 })

```

Note: in order to pass target accelerations into a charger feedforward, utilize the getTargetAccel lambda
passed into the Feedforward overloads. 





### Motion Profiling using SetpointSupplier

There are a couple of different setpoint suppliers:

A. SetpointSupplier.Default<I: AnyDimension, O: AnyDimension>(feedforward: Feedforward<I,O>


Example:

```kotlin
// uses Dimensions instead of Quantities
val positionController = SuperPIDController<AngleDimension, VoltageDimension>(
     pidConstants = PIDConstants(0.1,0.0,0.0),
     getInput = {encoder.angularPosition}, // lambda expression to fetch input; Quantity<AngleDimension> supplying function
     target = 5.0.radians, // Quantity<AngleDimension>
     setpointSupplier = ...,
     continuousInputRange = 0.degrees..360.degrees, // ClosedRange<T> is part of kotlin native; by default null
     outputRange = -12.volts..12.volts, // Automatically clamps output based off another ClosedRange; -infinity to infinity by default
     integralRange = ...,
     selfSustain = true // controls whether or not the controller is sustained every loop; defaults to false!
)

```

# Pose, Transform, and Translation 2d/3d

ChargerLib has typesafe units equivalents to all of these classes. The equivalents all start with the Unit prefix.
Conversion out and in these equivalents are also done with inUnit and ofUnit.

```kotlin
var pose2d = UnitPose2d(siValue = Pose2d(...))
pose2d = UnitPose2d(5.0.meters, 6.0.meters, 2.0.radians)

val pose2dWPI: Pose2d = pose2d.inUnit(meters) // converts angle component to Rotation2d, which itself is a typesafe system of measure
pose2d = pose2dWPI.ofUnit(meters)


var translation3d = UnitTranslation3d(...)
```

# Polynomial Equations

ChargerLib provides classes for polynomial equations that can be solved.
