/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Thor Eivind Andersen and Mats Rødseth (Master 2016 @ NTNU)
 */
package no.ntnu.tem.robot;

import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * This class represents each robot. It holds the robot's parameters and
 * characteristics. The class holds two different objects, IR and a
 * concurrentLinkedQueue of Measurements.
 *
 * The concurrentLinkedQueue "measurements" holds all the Measurements, done by
 * the robot, that is not yet processed by the program.
 *
 * @author Thor Eivind and Mats (Master 2016 @ NTNU)
 */
public class Robot {

    private final int id;
    private final String name;
    private final int width, length, axleOffset;
    private final int messageDeadline;
    private final int[] towerOffset, sensorOffset;
    private final IR irSensors;
    private final ConcurrentLinkedQueue<Measurement> measurements;

    private int[] initialPosition;
    private int[] estimatedPosition;
    private int robotOrientation;
    private boolean homeFlag;
    private final Object homeLock = new Object();
    private boolean goingHomeFlag;
    private final Object goingHomeLock = new Object();
    private boolean busyFlag;
    private final Object busyLock = new Object();
    private int[] destination;
    private int messageCorruptCount = 0;
    private int ValueCorruptCount = 0;

    /**
     * Constructor of the class Robot
     *
     * @param robotID The robots ID
     * @param name The robots name
     * @param width The robots physical width (cm)
     * @param length The robots physical length (cm)
     * @param messageDeadline The robots message deadline (min update rate[ms])
     * @param axleOffset Axle offset lengthwise
     * @param towerOffset Tower offset [a,b] a-lengthwise, b-crosswise
     * @param sensorOffset Sensor offset [a,b,c,d,..] from center (radius)
     * @param irHeading IR heading relative to 0 degrees (straight forward)
     */
    public Robot(int robotID, String name, int width, int length, int messageDeadline,
            int axleOffset, int[] towerOffset, int[] sensorOffset, int[] irHeading) {
        this.id = robotID;
        this.name = name;
        this.width = width;
        this.length = length;
        this.messageDeadline = messageDeadline;
        this.axleOffset = axleOffset;
        this.towerOffset = towerOffset;
        this.sensorOffset = sensorOffset;
        this.irSensors = new IR(irHeading);
        this.measurements = new ConcurrentLinkedQueue<>();

        this.initialPosition = new int[]{0, 0, 0};
        this.estimatedPosition = new int[]{0, 0};
        this.destination = new int[]{0, 0};
    }

    /**
     * Method that sets the robots initial position
     *
     * @param x the robots x-value
     * @param y the robots y-value
     * @param orientation the robots orientation
     */
    public void setInitialPosition(int x, int y, int orientation) {
        initialPosition = new int[]{x, y, orientation};
    }

    /**
     * Method that returns the robots initial position
     *
     * @return the robots initial x, y and orientation
     */
    public int[] getInitialPosition() {
        return this.initialPosition;
    }

    /**
     * Puts a Measurement at the end of the measurement queue. This method is
     * thread safe and will never block. It can throw a NullPointerExeption if
     * some of the values are wrong or null, if not it will return true.
     *
     * @param measuredOrientation Measured theta
     * @param measuredPosition Measured position as an int[] x first, then y
     * @param towerHeading the heading of the first sensor in the ir tower
     * @param irData the ir data that where taken at the same time.
     * @return true if successful, it may throw an exception if some parameters
     * are wrong or null
     */
    public boolean addMeasurement(int measuredOrientation, int[] measuredPosition, int towerHeading, int[] irData) {
        int[] irHeading = new int[irData.length];
        for (int i = 0; i < irData.length; i++) {
            irHeading[i] = (towerHeading + irSensors.getSpreading()[i]) % 360;
        }
        Measurement measurment = new Measurement(measuredOrientation, measuredPosition, irHeading, irData);
        return measurements.offer(measurment);
    }

    /**
     * Returns and removes the oldest measurement done by the robot, this method
     * is thread safe and will return null if there are no more measurements
     * left.
     *
     * @return the oldest measurement done by this robot or null if there are no
     * measurements in the queue.
     */
    public Measurement getMeasurement() {
        Measurement m = measurements.poll();
        return m;
    }

    /**
     * Method that returns the robots id
     *
     * @return the id
     */
    public int getId() {
        return id;
    }

    /**
     * Method that returns the robots name
     *
     * @return the name
     */
    public String getName() {
        return name;
    }

    /**
     * Method that returns the robots position
     *
     * @return the position
     */
    public int[] getPosition() {
        return estimatedPosition;
    }

    public void setPosition(int[] position) {
        this.estimatedPosition = position;
    }

    /**
     * Method that returns the robots orientation
     *
     * @return the orientation
     */
    public int getRobotOrientation() {
        return robotOrientation;
    }

    public void setRobotOrientation(int robotOrientation) {
        this.robotOrientation = robotOrientation;
    }

    /**
     * Method that returns the robots physical width
     *
     * @return the width
     */
    public int getWidth() {
        return width;
    }

    /**
     * Method that returns the robots physical length
     *
     * @return the length
     */
    public int getLength() {
        return length;
    }

    /**
     * Method that returns the robots tower offset
     *
     * @return the offset
     */
    public int[] getTowerOffset() {
        return towerOffset;
    }

    /**
     * Method that returns the robots sensor offset
     *
     * @return the sensor offset
     */
    public int[] getSensorOffset() {
        return sensorOffset;
    }

    /**
     * Method that returns the robots axle offset
     *
     * @return the axle offset
     */
    public int getAxleOffset() {
        return axleOffset;
    }

    /**
     * Method that returns the robots ir sensors
     *
     * @return the sensors
     */
    public IR getIRSensors() {
        return irSensors;
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

    
    /**
     * Method that returns the robots goingHomeFlag
     * 
     * @return true if the robot is going home
     */
    public boolean isGoingHome(){
        synchronized (goingHomeLock){
            return goingHomeFlag;
        }
    }
    /**
     * Method that sets the robot to status going home
     *
     * @param goingHomeFlag true if the robot is going home
     */
    
    public void setGoingHome(boolean goingHomeFlag){
        synchronized (goingHomeLock) {
            this.goingHomeFlag = goingHomeFlag;
        }
    }
    
      public boolean isHome(){
        synchronized (homeLock){
            return homeFlag;
        }
    }
    /**
     * Method that sets the robot to status going home
     *
     * @param homeFlag true if the robot is going home
     */
    
    public void setHome(boolean homeFlag){
        synchronized (homeLock) {
            this.homeFlag = homeFlag;
        }
    }
    /**
     * Method that returns the robots destination
     *
     * @return the destination
     */
    public int[] getDestination() {
        return destination;
    }

    /**
     * Method that updates the robots destination
     *
     * @param destination the robots destination (x,y)
     */
    public void setDestination(int[] destination) {
        this.destination = destination;
    }

    /**
     * Method that increment the message corrupt counter
     *
     */
    public void addMessageCorruptCount() {
        this.messageCorruptCount++;
    }

    /**
     * Method that increment the value corrupt counter
     *
     */
    public void addValueCorruptCount() {
        this.ValueCorruptCount++;
    }

    /**
     * Method that returns the robots number of received corrupt messages/values
     *
     * @return the counter
     */
    public int getCorruptCount() {
        return messageCorruptCount + ValueCorruptCount;
    }
}
