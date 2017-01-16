/**
 * This code is written as part of a Master Thesis
 * the spring of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 * 
 * Based on BasicRobot.java
 */
package no.ntnu.et.simulator;

import java.util.concurrent.LinkedBlockingQueue;
import no.ntnu.et.general.Pose;

/**
 *
 * @author geirhei
 */
public class SlamRobot extends SimRobot {
    private int[][] mapWindow;
    private LinkedBlockingQueue<int[]> measurementQueue;
    
    public SlamRobot(SimWorld world, Pose initialPose, String name, int id) {
        super(world, initialPose, name, id);
        mapWindow = new int[100][100];
        measurementQueue = new LinkedBlockingQueue(5);
    }
    
    /**
     * Adds a measurement to the internal measurementQueue.
     * 
     * @param meas
     * @return True if successful. Throws IllegalStateException is queue is full.
     */
    boolean addMeasurement(int[] meas) {
        try {
            return measurementQueue.add(meas);
        } catch (Exception e) {
            System.err.println("Exception in addMeasurement: " + e.getMessage());
            return false;
        }
    }
}
