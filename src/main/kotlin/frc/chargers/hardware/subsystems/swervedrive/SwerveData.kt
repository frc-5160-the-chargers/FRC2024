package frc.chargers.hardware.subsystems.swervedrive

import frc.chargers.hardware.configuration.ConfigurableHardware
import frc.chargers.hardware.configuration.HardwareConfiguration
import kotlin.reflect.full.primaryConstructor

/**
 * Creates a [SwerveData] instance, where all members can be configured.
 * For this function to work, the [SwerveData] instance must hold values
 * that implement [ConfigurableHardware], with the appropriate [HardwareConfiguration].
 */
inline fun <reified C: HardwareConfiguration, T: ConfigurableHardware<C>> SwerveData( // reified T allows the ::class call to work, while the inline modifier allows the configure function to be inlined(improving performance).
    topLeft: T,
    topRight: T,
    bottomLeft: T,
    bottomRight: T,
    configure: C.() -> Unit
): SwerveData<T> {
    val primaryConstructor = C::class.primaryConstructor ?: error("The configuration class " + C::class.simpleName + " must have a constructor.")
    try{
        val configuration = primaryConstructor.callBy(emptyMap()).apply(configure)

        topLeft.configure(configuration)
        topRight.configure(configuration)
        bottomLeft.configure(configuration)
        bottomRight.configure(configuration)

        return SwerveData(topLeft, topRight, bottomLeft, bottomRight)
    }catch(e: Exception){
        error(
            "A configuration class must have a primary constructor with only default parameters; " +
            "however, " + C::class.simpleName + " does not."
        )
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
): List<T> by listOf(topLeft, topRight, bottomLeft, bottomRight) { // The by operator allows us to automatically implement List<T>'s required methods through an instance of List<T>(which here, is created through listOf(topLeft, ...)

    companion object{
        /**
         * Creates a [SwerveData] instance where the topLeft, topRight, bottomLeft and bottomRight values
         * are generated through a function([creator]).
         */
        inline fun <T> create(creator: (Int) -> T): SwerveData<T> =
            SwerveData(creator(0), creator(1), creator(2), creator(3))
    }

    /**
     * Maps each value within the [SwerveData] instance to another value.
     */
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
}