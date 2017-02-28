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
import no.ntnu.ge.slam.LocalMap;
import no.ntnu.tem.robot.IR;

/**
 *
 * @author geirhei
 */
public class SlamRobot extends SimRobot {
    private final int windowHeight = 50;
    private final int windowWidth = 50;
    private LocalMap localMap;
    private LinkedBlockingQueue<int[]> updateQueue;
    private boolean busyFlag = false;
    private final Object busyLock = new Object();
    private int[] irHeading;
    private int lineOfSight = 40; //cm
    private MapLocation globalRobotLocation;
    private MapLocation localRobotLocation;
    
    SlamRobot(SimWorld world, Pose initialPose, String name, int id) {
        super(world, initialPose, name, id);
        localMap = new LocalMap(windowHeight, windowWidth);
        updateQueue = new LinkedBlockingQueue<>(5);
        irHeading = new int[super.getLastIrMeasurement().length];
        globalRobotLocation = new MapLocation((int) initialPose.getPosition().getYValue(), (int) initialPose.getPosition().getXValue());
        localRobotLocation = new MapLocation(windowHeight/2-1, windowWidth/2-1);
    }
    
    void updateIrHeading() {
        int[] sensors = {0, 90, 180, 270};
        IR irSensors = new IR(sensors);
        for (int i = 0; i < super.getLastIrMeasurement().length; i++) {
            irHeading[i] = ((int) super.getTowerAngle().getValue() + irSensors.getSpreading()[i]) % 360;
        }
    }
    
    public int getLineOfSight() {
        return lineOfSight;
    }
    
    public MapLocation getGlobalRobotLocation() {
        return globalRobotLocation;
    }
    
    public void setGlobalRobotLocation(MapLocation location) {
        globalRobotLocation = location;
    }
    
    public MapLocation getLocalRobotLocation() {
        return localRobotLocation;
    }
    
    public LocalMap getWindowMap() {
        return localMap;
    }
    
    public LinkedBlockingQueue<int[]> getUpdateQueue() {
        return updateQueue;
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
