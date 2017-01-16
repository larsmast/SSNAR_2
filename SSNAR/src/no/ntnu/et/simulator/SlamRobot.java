/**
 * This code is written as part of a Master Thesis
 * the spring of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 * 
 * Based on SimRobot.java
 */
package no.ntnu.et.simulator;

import no.ntnu.et.general.Pose;

/**
 *
 * @author geirhei
 */
public class SlamRobot extends SimRobot {
    private int[][] mapWindow;
    
    public SlamRobot(SimWorld world, Pose initialPose, String name, int id) {
        super(world, initialPose, name, id);
        
        mapWindow = new int[100][100];
    }
}
