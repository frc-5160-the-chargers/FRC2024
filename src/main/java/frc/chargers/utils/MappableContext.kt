@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.utils



/**
 * A context class used to create MutableMap's
 * using lambda blocks.
 */
public class MappableContext<K,V> {
    public var map: MutableMap<K,V> = mutableMapOf()

    public infix fun K.mapsTo(other: V){
        map[this] = other
    }
}