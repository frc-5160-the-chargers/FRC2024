package frc.robot

/*
/**
 * An example of a property with a custom getter and a custom setter.
 *
 * When this property is assigned a value, the code within the set(){ }
 * block will also run.
 *
 * Note that this property does not store any data,
 * and is essentially a proxy for 2 different methods.
 */
var exampleGetSetProperty: Double
    get() = motor.get()
    set(value) {
        if (digitalInput.get()) motor.set(value)
    }

/**
 * An example of a getter/setter property with a backing field.
 *
 * A backing field acts like an invisible variable tied to the property itself,
 * which can be accessed in the get() and set() methods of that property.
 *
 * In order to use a backing field, an initial value must be given to the property
 * itself.
 *
 * To refer to a backing field, use the keyword "field".
 *
 * Properties with a backing field can exclude either the getter or the setter.
 */
var exampleBackingFieldProperty = exampleConstructorArg
    get() {
        field++
        return field
    }
    set(value) {
        motor.stopMotor()
        field = value + 1
    }

 */