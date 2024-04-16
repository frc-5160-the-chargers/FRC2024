package frc.robot

import edu.wpi.first.util.datalog.StringLogEntry
import edu.wpi.first.wpilibj.DataLogManager
import frc.chargers.framework.ChargerRobot
import frc.debugging.EmptyRobotContainer
import monologue.Annotations.IgnoreLogged
import monologue.Logged
import monologue.Monologue
import kotlin.reflect.KClass
import kotlin.reflect.KProperty0

class Robot: ChargerRobot(

)

object TestRobot: Logged{

    class Data<T>(val data: T)
    inline fun <reified T : Any?, S : T?> Data<S>.kclass(): KClass<T> = T::class


    init{
        log("hi", 2.0)
        Monologue.setupMonologue(this, "Robot", false, true)

        val data = Data<Double?>(2.0)

        val hi = data.kclass()

        val test: Double? = 2.0
        val hello = test!!::class.java
        val test2: Double = 1.0
        val hello2: Class<Double> = test2::javaClass.get()


        val stringLogEntry = StringLogEntry(DataLogManager.getLog(), "Hi")
        stringLogEntry.append("Hi")
        DataLogManager.getLog()
    }
}
