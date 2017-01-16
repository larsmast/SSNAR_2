/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.simulator;

import no.ntnu.et.general.Pose;

/**
 * This class represents the virtual robots in the simulator. It has private
 * variables to represent the state of the robots such as the pose
 * The robots behavior is created by calling methods in this class such as
 * moveRobot().
 * 
 * @author Eirik Thon
 */
public class BasicRobot extends SimRobot {
    
    /**
     * Constructor for Robot.
     * @param initialPose
     * @param name
     * @param id
     * @param world
     */
    public BasicRobot(SimWorld world, Pose initialPose, String name, int id) {
        super(world, initialPose, name, id);
    }
    
}
