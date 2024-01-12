@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.wpilibextensions

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.micro
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj.Timer
import org.littletonrobotics.junction.Logger

public fun fpgaTimestamp(): Time = Timer.getFPGATimestamp().ofUnit(seconds)

public fun fpgaTimestampReal(): Time = Logger.getRealTimestamp().ofUnit(micro.seconds)

public fun timeSinceMatchStart(): Time = Timer.getMatchTime().ofUnit(seconds)

public fun delay(time: Time){
    Timer.delay(time.inUnit(seconds))
}



