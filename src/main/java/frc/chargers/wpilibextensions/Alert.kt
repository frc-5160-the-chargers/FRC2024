@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.wpilibextensions

import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import monologue.Logged
import java.nio.ByteBuffer


/**
 * Credits: [Team 4099 repository](https://github.com/team4099/ChargedUp-2023/tree/main)
 */
public class Alert(group: String = "Alerts", text: String, type: AlertType) {
    public companion object {
        private val groups: MutableMap<String, SendableAlerts> = mutableMapOf()

        /**
         * A convenience function to create an [Alert] with the [AlertType] of Error.
         */
        public fun error(group: String = "Alerts", text: String): Alert = Alert(group,text, AlertType.ERROR)
        /**
         * A convenience function to create an [Alert] with the [AlertType] of Warning.
         */
        public fun warning(group: String = "Alerts", text: String): Alert = Alert(group,text, AlertType.WARNING)
        /**
         * A convenience function to create an [Alert] with the [AlertType] of Info.
         */
        public fun info(group: String = "Alerts", text: String): Alert = Alert(group,text, AlertType.INFO)
    }


    private val type: AlertType

    private var activeStartTime = 0.0

    public var text: String

    /**
     * Sets whether the alert should currently be displayed. When activated, the alert text will also
     * be sent to the console.
     */
    public var active: Boolean = false
        set(active){
            if (active && !this.active) {
                activeStartTime = Timer.getFPGATimestamp()
                when (type) {
                    AlertType.ERROR -> DriverStation.reportError(text, false)
                    AlertType.WARNING -> DriverStation.reportWarning(text, false)
                    AlertType.INFO -> println(text)
                }
            }
            field = active
        }





    private class SendableAlerts : Sendable {
        val alerts: MutableList<Alert> = mutableListOf()
        fun getStrings(type: AlertType): List<String> = alerts
                .filter { x: Alert -> x.type == type && x.active }
                .sortedBy { it.activeStartTime }
                .map { a: Alert -> a.text }

        override fun initSendable(builder: SendableBuilder) {
            builder.setSmartDashboardType("Alerts")
            builder.addStringArrayProperty("errors", { getStrings(AlertType.ERROR).toTypedArray() }, null)
            builder.addStringArrayProperty(
                "warnings", { getStrings(AlertType.WARNING).toTypedArray() }, null
            )
            builder.addStringArrayProperty("infos", { getStrings(AlertType.INFO).toTypedArray() }, null)
        }
    }

    /** Represents an alert's level of urgency. */
    public enum class AlertType {
        /**
         * High priority alert - displayed first on the dashboard with a red "X" symbol. Use this type
         * for problems which will seriously affect the robot's functionality and thus require immediate
         * attention.
         */
        ERROR,

        /**
         * Medium priority alert - displayed second on the dashboard with a yellow "!" symbol. Use this
         * type for problems which could affect the robot's functionality but do not necessarily require
         * immediate attention.
         */
        WARNING,

        /**
         * Low priority alert - displayed last on the dashboard with a green "i" symbol. Use this type
         * for problems which are unlikely to affect the robot's functionality, or any other alerts
         * which do not fall under "ERROR" or "WARNING".
         */
        INFO
    }

    /**
     * Creates a new Alert. If this is the first to be instantiated in its group, the appropriate
     *
     * entries will be added to NetworkTables.
     */
    init {
        if (!groups.containsKey(group)) {
            groups[group] = SendableAlerts()
            SmartDashboard.putData(group, groups[group])
        }

        this.text = text
        this.type = type
        groups[group]?.alerts?.add(this)
    }
}