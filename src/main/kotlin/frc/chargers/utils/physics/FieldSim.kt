package frc.chargers.utils.physics

import com.batterystaple.kmeasure.quantities.Mass
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.grams
import com.batterystaple.kmeasure.units.kilo
import com.batterystaple.kmeasure.units.meters
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import org.dyn4j.dynamics.Body
import org.dyn4j.geometry.Geometry
import org.dyn4j.geometry.MassType
import org.dyn4j.world.World

/**
 * A wrapper class around [org.dyn4j]
 * that interfaces with ChargerLib classes.
 *
 * This allows for realistic collisions with robots,
 * gamepieces, and field elements.
 */
class FieldSim(
    private val drivetrainData: Map<Mass, EncoderHolonomicDrivetrain> = mapOf(),
){
    private val world = World<Body>()

    init {
        for ((mass, drivetrain) in drivetrainData) {
            val trackWidth = drivetrain.constants.trackWidth.inUnit(meters)
            val wheelBase = drivetrain.constants.wheelBase.inUnit(meters)
            val body = Body()
            body.setMass(MassType.NORMAL)
            body.addFixture(
                Geometry.createRectangle(trackWidth, wheelBase),
                mass.inUnit(kilo.grams) / (trackWidth * wheelBase)
            )
            body.linearDamping = 0.05
            body.angularDamping = 0.05
            world.addBody(body)
        }
    }
}