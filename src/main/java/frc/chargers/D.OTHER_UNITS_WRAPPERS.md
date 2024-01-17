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

Feedforward control is handled through the ```Feedforward<I: AnyDimension, O: AnyDimension>``` interface.


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
