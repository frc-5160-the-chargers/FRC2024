# ChargerLib Basics: Kmeasure

ChargerLib designed to integrate with the [Kmeasure Units Library](https://github.com/battery-staple/KMeasure). 
For more information, check the library's README page. 

## Basics:

A. All values are templated off of the ```Quantity<D: Dimension>``` class. This class has only one property: 'siValue', which is the base value of the quantity in the International System of Unit's designated base unit for the class. Due to being inline value classes, ```Quantity<D>```'s are represented as Double's as much as possible; this significantly reduces runtime overhead(and removes the need of a MutableQuantity/MutableMeasure class).
B. The Dimension class represents a generic unit/dimension, and is used to distinguish between different ```Quantity```'s. It is represented using 4 components: Mass(M), Length(L), Time(T) and Current(I). For instance:
```kotlin
typealias dimensionA =  Dimension<Mass0, Length0, Time1, Current0> // dimension = time
typealias dimensionB =  Dimension<Mass1, LengthN1, Time2, Current1> // represents mass^1 * length^-1 * time^2 * current^1
``` 
Dimension is a sealed class; thus, it can only be passed as a type argument, and cannot be instantiated by itself.
In order to represent a generic dimension, use ```Dimension<*,*,*,*>``` as a type bound.
Note: At the moment, the k2 compiler has issues detecting 

C. There are named typealiases for common dimensions and Quantity's. For instance:
```kotlin
typealias TimeDimension = Dimension<Mass0, Length0, Time1, Current0>
typealias Time = Quantity<TimeDimension>

// instead of doing this
val time: Quantity<Dimension<Mass0, Length0, Time1, Current0>> = Quantity(0.0)
// you can now do this:
val time: Time = Time(0.0)
// or this:
val time: Quantity<TimeDimension> = Quantity(0.0)
```
D. To convert a Int, Long, or Double into a Quantity, you can either use ```ofUnit```, or an extension property for base units:
```
val time = 5.ofUnit(seconds)
val time2 = 5.seconds // uses extension property of Double; equivalent to 5.ofUnit(seconds)

val time3 = Time(5.0) // Creates a new Quantity with 5.0 as the si value.

val velocity = 5.ofUnit(meters / seconds)
val velocity2 = 5.meters / 1.seconds
val velocity3 = Velocity(5.0)
```

E. To convert out of a Quantity, use the inUnit extension function:

```kotlin
val timeAsSecs = Time(5.0).inUnit(seconds)
val timeAsHours = Time(5.0).inUnit(hours)
```

F. Units are treated the same as Quantity's

In Kmeasure, a Unit(I.E meters, seconds, etc.) are actually represented by ```Quantity<D>```'s internal, with the quantity representing the conversion factor of the unit from the SI unit.

In order to increase code clarity, ChargerLib adds the ```KmeasureUnit<D: Dimension>``` typealias.


## Wrappers that utilize Kmeasure:

A. Hardware 