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
import no.ntnu.et.general.Position;
import no.ntnu.et.map.MapLocation;
import no.ntnu.ge.slam.WindowMap;

/**
 *
 * @author geirhei
 */
public class SlamRobot extends SimRobot {
    private final int windowHeight = 100;
    private final int windowWidth = 50;
    private WindowMap windowMap;
    private LinkedBlockingQueue<int[]> updateQueue;
    private boolean busyFlag = false;
    private final Object busyLock = new Object();
    private boolean inWallCollision = false;
    private int robotOrientation;
    private MapLocation robotWindowLocation;
    
    SlamRobot(SimWorld world, Pose initialPose, String name, int id) {
        super(world, initialPose, name, id);
        robotOrientation = (int) initialPose.getHeading().getValue();
        windowMap = new WindowMap(windowHeight, windowWidth, robotOrientation);
        updateQueue = new LinkedBlockingQueue<>(5);
        robotWindowLocation = new MapLocation(windowHeight/4, windowWidth/2);
    }
    
    public MapLocation getRobotWindowLocation() {
        return robotWindowLocation;
    }
    
    public void setRobotWindowLocation(MapLocation location) {
        robotWindowLocation = location;
    }
    
    public void setRobotOrientation(int orientation) {
        robotOrientation = orientation;
    }
    
    public int getRobotOrientation() {
        return robotOrientation;
    }
    
    public WindowMap getWindowMap() {
        return windowMap;
    }
    
    public LinkedBlockingQueue<int[]> getUpdateQueue() {
        return updateQueue;
    }
    
    public boolean getInWallCollision() {
        return inWallCollision;
    }
    
    public void setInWallCollision(boolean bool){
        inWallCollision = bool;
    }
    
    public boolean isInCollisionManagement() {
        return inWallCollision; // Maybe add robotCollision here if implemented
    }
    
    /**
     * Adds a measurement to the internal updateQueue.
     * 
     * @param meas
     * @return True if successful. Throws IllegalStateException is queue is full.
     */
    boolean addUpdate(int[] meas) {
        try {
            return updateQueue.add(meas);
        } catch (Exception e) {
            System.err.println("Exception in addUpdate: " + e.getMessage());
            return false;
        }
    }
    
    /**
     * Returns and removes an update from the internal updateQueue.
     * 
     * @return int[] if successful, null if no elements in queue
     */
    public int[] getUpdate() {
        return updateQueue.poll();
    }
    
    public MapLocation getInitialLocation() {
        Pose initialPose = getInitialPose();
        Position initialPosition = initialPose.getPosition();
        return new MapLocation((int)initialPosition.getXValue(), (int)initialPosition.getYValue());
    }
    
    /**
     * Method that returns the robots busyflag
     *
     * @return true if the robot is busy
     */
    public boolean isBusy() {
        synchronized (busyLock) {
            return busyFlag;
        }
    }

    /**
     * Method that sets the robots status to busy
     *
     * @param busyFlag true if robot is busy
     */
    public void setBusy(boolean busyFlag) {
        synchronized (busyLock) {
            this.busyFlag = busyFlag;
        }
    }
}
