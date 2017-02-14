/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 * Modified by Geir Eikeland (Master 2017 @ NTNU)
 */
package no.ntnu.et.mapping;

import no.ntnu.et.general.Position;

/**
 * This class is used to store the position of an IR-measurement.
 * 
 * @author Eirik Thon
 */
public class Sensor {
    Position position;
    Position offsetPosition;
    boolean measurement;

    public Sensor() {
    }
    
    /* Unused
    public Sensor(Position position, boolean measurement) {
        this.position = position;
        this.measurement = measurement;
    }
    */

    public Position getPosition() {
        return position;
    }
    
    public Position getOffsetPosition() {
        return offsetPosition;
    }

    public boolean isMeasurement() {
        return measurement;
    }
    
    public void setPosition(Position position) {
        this.position = position;
    }
    
    public void setOffsetPosition(Position position) {
        this.offsetPosition = position;
    }

    public void setMeasurement(boolean measurement) {
        this.measurement = measurement;
    }
}