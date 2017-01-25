/**
 * This code is written as part of a Master Thesis
 * the spring of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 */
package no.ntnu.ge.slam;

import no.ntnu.et.general.Angle;
import no.ntnu.et.general.Pose;
import no.ntnu.et.general.Position;
import no.ntnu.et.mapping.Sensor;
import no.ntnu.et.simulator.SlamRobot;

/**
 * This class is used to find the location of a robot of type SlamRobot and its
 * IR-measurements in the map. Based on no.ntnu.et.mapping
 * 
 * @author Geir Eikeland
 */
class SlamMeasurementHandler {
    private final Pose initialPose;
    private final int sensorRange = 40;
    private int[] currentMeasurement;
    private Pose robotPose;
    private Sensor[] sensors;
    private SlamRobot robot;
    
    /**
     * Constructor for class SlamMeasurementHandler
     * 
     * @param robot SlamRobot
     * @param initialPose Pose
     */
    SlamMeasurementHandler(SlamRobot robot, Pose initialPose) {
        this.initialPose = initialPose;
        this.robot = robot;
        sensors = new Sensor[4];
        for (int i = 0; i < 4; i++) {
            sensors[i] = new Sensor();
        }
    }
    
    boolean updateMeasurement() {
        currentMeasurement = robot.getMeasurement();
        if (currentMeasurement == null) {
            return false;
        }
        Position robotPosition = new Position((double)currentMeasurement[0], (double)currentMeasurement[1]);
        Angle robotAngle = new Angle(currentMeasurement[2]);
        robotPose = new Pose(robotPosition, robotAngle);
        robotPose.transform(this.initialPose);
        
        // Update sensor data
        int[] irData = {currentMeasurement[4],currentMeasurement[5],currentMeasurement[6],currentMeasurement[7]};
        int[] irHeading = {currentMeasurement[3]};
        for (int i = 0; i < 4; i++) {
            int measurementDistance = irData[i];
            if (measurementDistance == 0 || measurementDistance > sensorRange) {
                sensors[i].setMeasurement(false);
                measurementDistance = sensorRange;
            }else{
                sensors[i].setMeasurement(true);
            }
            Angle towerAngle  = new Angle((double)irHeading[i]);
            Angle sensorAngle = Angle.sum(towerAngle, robotPose.getHeading());
            double xOffset = measurementDistance * Math.cos(Math.toRadians(sensorAngle.getValue()));
            double yOffset = measurementDistance * Math.sin(Math.toRadians(sensorAngle.getValue()));
            Position measurementPosition = Position.sum(robotPose.getPosition(), new Position(xOffset, yOffset));
            sensors[i].setPosition(measurementPosition);
        }
        return true;
    }
}
