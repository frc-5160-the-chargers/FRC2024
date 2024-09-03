package frc.chargers.hardware.subsystems.swervedrive

/**
 * Represents generic data useful to an [EncoderHolonomicDrivetrain];
 * with values corresponding to the top left, top right, bottom left and bottom right modules of the drivetrain.
 *
 * This can hold: Encoders, Motors, output voltage, output current, etc.
 *
 * This class implements the [List] interface, allowing it to call the same functions that a list can(like forEach, map, etc.)
 * along with being substituted for a list.
 */
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