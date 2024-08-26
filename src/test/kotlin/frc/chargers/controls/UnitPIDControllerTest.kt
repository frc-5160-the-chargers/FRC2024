package frc.chargers.controls

import com.batterystaple.kmeasure.dimensions.ScalarDimension
import com.batterystaple.kmeasure.quantities.Quantity
import edu.wpi.first.hal.HAL
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import edu.wpi.first.math.controller.PIDController
import frc.chargers.utils.math.equations.epsilonEquals


class UnitPIDControllerTest {

    @BeforeEach
    fun setUp() {
        assert(HAL.initialize(500,0))
    }

    @Test
    fun calculate() {
        val wpiController = PIDController(0.1, 0.0, 0.1)
        val chargerController = UnitPIDController<ScalarDimension, ScalarDimension>(0.1, 0.0, 0.1)

        for (i in 0..10){
            assert(
                wpiController.calculate(5.0, 0.0) epsilonEquals
                chargerController.calculate(Quantity(5.0), Quantity(0.0)).siValue
            )
            Thread.sleep(20)
        }
    }
}