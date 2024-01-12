@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.utils.math.units

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.dimensions.*

public typealias VoltageRateDimension = Dimension<Mass1,Length2,TimeN4,CurrentN1>

public typealias VoltageRate = KmeasureUnit<VoltageRateDimension>



public typealias VoltagePerAngleDimension = ElectricalPotentialDimension

public typealias VoltagePerAngle = KmeasureUnit<VoltagePerAngleDimension>