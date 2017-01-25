/**
 * This code is written as part of a Master Thesis
 * the spring of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 */
package no.ntnu.ge.slam;

import no.ntnu.et.general.Pose;
import no.ntnu.et.mapping.Sensor;
import no.ntnu.et.simulator.SlamRobot;
import no.ntnu.tem.robot.Measurement;

/**
 *
 * @author geirhei
 */
public class SlamMeasurementHandler {
    private final Pose initialPose;
    private final int sensorRange = 40;
    private Measurement currentMeasurement;
    private Pose robotPose;
    private Sensor[] sensors;
    private SlamRobot robot;
    
    public SlamMeasurementHandler(SlamRobot robot, Pose initialPose) {
        this.initialPose = initialPose;
        this.robot = robot;
        sensors = new Sensor[4];
        for (int i = 0; i < 4; i++) {
            sensors[i] = new Sensor();
        }
    }
    
    boolean updateMeasurement() {
        
    }
}
