package frc.chargers.hardware.subsystems.swervedrive

import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.HardwareConfiguration


/**
 * Creates a [SwerveData] instance, where all members can be configured.
 */
inline fun <reified C: HardwareConfiguration, T: HardwareConfigurable<C>> SwerveData(
    topLeft: T,
    topRight: T,
    bottomLeft: T,
    bottomRight: T,
    configure: C.() -> Unit
): SwerveData<T> {
    try{
        val configuration = C::class.constructors.first().call().apply{ configure() }

        topLeft.configure(configuration)
        topRight.configure(configuration)
        bottomLeft.configure(configuration)
        bottomRight.configure(configuration)

        return SwerveData(topLeft, topRight, bottomLeft, bottomRight)
    }catch(e: Exception){
        error("It looks like the configuration class " + C::class.simpleName + " does not have a no-args constructor. All configuration classes must idiomatically have a no-args constructor.")
    }
}


/**
 * Represents generic data useful to an [EncoderHolonomicDrivetrain];
 * with values corresponding to the top left, top right, bottom left and bottom right modules of the drivetrain.
 *
 * This can hold: Encoders, Motors, output voltage, output current, etc.
 *
 * This class implements the [List] interface, allowing it to call the same functions that a list can(like forEach, map, etc.)
 * along with being substituted for a list.
 */
@Suppress("unused")
data class SwerveData<out T>(
    val topLeft: T,
    val topRight: T,
    val bottomLeft: T,
    val bottomRight: T
): List<T> by listOf(topLeft, topRight, bottomLeft, bottomRight) { // The by operator allows us to automatically implement List<T>'s required functions through an instance of List<T>(which here, is created through listOf(topLeft, ...)
    inline fun <reified C> contains(): Boolean =
        topLeft is C && topRight is C && bottomLeft is C && bottomRight is C

    inline fun <C> map(mapper: (T) -> C): SwerveData<C> =
        SwerveData(
            mapper(topLeft),
            mapper(topRight),
            mapper(bottomLeft),
            mapper(bottomRight)
        )

    /**
     * Zips 2 SwerveData classes together into one,
     * which holds pairs of values.
     */
    fun <S> zip(otherData: SwerveData<S>): SwerveData<Pair<T, S>> =
        SwerveData(
            Pair(this.topLeft, otherData.topLeft),
            Pair(this.topRight, otherData.topRight),
            Pair(this.bottomLeft, otherData.bottomLeft),
            Pair(this.bottomRight, otherData.bottomRight)
        )

    companion object{
        inline fun <T> generate(creator: (Int) -> T): SwerveData<T> =
            SwerveData(creator(0), creator(1), creator(2), creator(3))
    }
}