/**
 * This code is written as part of a Master Thesis
 * the spring of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 */
package no.ntnu.et.simulator;

import no.ntnu.et.general.Angle;
import no.ntnu.et.general.Line;
import no.ntnu.et.general.Pose;
import no.ntnu.et.general.Position;
import no.ntnu.et.general.Utilities;

/**
 *
 * @author geirhei
 */
abstract public class SimRobot {
    private SimWorld world;
    private Pose pose;
    private Pose estimatedPose;
    final private Pose initialPose;
    private String name;
    private int id;
    private double measuredDistance;
    private double measuredRotation;
    private double targetDistance;
    private double targetRotation;
    private int rotationDirection;
    private int movementDirection;
    private boolean rotationFinished;
    private boolean translationFinished;
    private Angle towerAngle;
    final private double turnSpeed = 0.5;
    final private double moveSpeed = 0.1;
    final private double towerSpeed = 0.25;
    private int towerDirection;
    final private Object movementLock = new Object();
    final private double maxVisualLength = 80;
    final private double minVisualLength = 10;
    private double[] lastIrMeasurement;
    private Position targetPosition;
    private int diameter = 10;
    
    public SimRobot(SimWorld world, Pose initialPose, String name, int id) {
        this.world = world;
        towerDirection = 1;
        towerAngle = new Angle(0);
        this.name = name;
        this.id = id;
        pose = initialPose;
        this.initialPose = Pose.copy(initialPose);
        estimatedPose = new Pose(0, 0, 0);
        targetDistance = 0;
        targetRotation = 0;
        measuredDistance = 0;
        measuredRotation = 0;
        lastIrMeasurement = new double[4];
        rotationDirection = 1;
        movementDirection = 1;
        rotationFinished = true;
        translationFinished = true;       
        targetPosition = Position.copy(pose.getPosition());
    }
    
    public int getRobotOrientation() {
        return (int) pose.getHeading().getValue();
    }
    
    public double getMaxSensorDistance(){
        return maxVisualLength;
    }
    
    Angle getTowerAngle() {
        return towerAngle;
    }
    
    double[] getLastIrMeasurement() {
        return lastIrMeasurement;
    }
    
    /**
     * Returns the ID of the robot
     * @return integer
     */
    int getId() {
        return id;
    }
    
    /**
     * Returns the name of the robot
     * @return String
     */
    String getName() {
        return name;
    }
    
    /**
     * Sets the target rotation and target heading. Also resets the measured
     * rotation and distance.
     * @param theta
     * @param distance 
     */
    public void setTarget(double theta, double distance) {
        synchronized(movementLock) {
            targetRotation = theta;
            targetDistance = distance;
            measuredRotation = 0;
            measuredDistance = 0;
            rotationDirection = (int)Math.signum(theta);
            movementDirection = (int)Math.signum(distance);
            Angle targetAngle = Angle.sum(pose.getHeading(), new Angle(theta));
            Position offset = Utilities.polar2cart(targetAngle, distance);
            targetPosition = Position.sum(pose.getPosition(), offset);
            rotationFinished = false;
            translationFinished = false;
        }
    }
    
    /**
     * Creates and returns a copy of the pose
     * @return 
     */
    public Pose getPose() {
        return pose;
    }
    
    public Pose getInitialPose() {
        return initialPose;
    }
    
    /**
     * Creates and returns a copy of the estimated pose
     * @return 
     */
    Pose getEstimatedPose() {
        return estimatedPose;
    }
    
    Position getTargetPosition(){
        return targetPosition;
    }
    
    public boolean isRotationFinished() {
        return rotationFinished;
    }
    
    /**
     * Turns the tower 5 degrees within a 90 degree angle.
     */
    void turnTower() {
        int clockWise = -1;
        int counterClockWise = 1;
        if (towerAngle.getValue() < towerSpeed && towerDirection == clockWise) {
            towerDirection = counterClockWise;
        } else if (towerAngle.getValue() >= 90 && towerDirection ==  counterClockWise) {
            towerDirection = clockWise;
        }
        towerAngle.add(towerDirection * towerSpeed);
    }
    
    /**
     * Creates a package of all measurements with a timestamp. The package is 
     * sent to the application and also stored in the measurement history of the
     * robot.
     */
    int[] createMeasurement() {
        int[] measurement = new int[8];
        measurement[0] = (int)Math.round(estimatedPose.getPosition().getXValue());
        measurement[1] = (int)Math.round(estimatedPose.getPosition().getYValue());
        measurement[2] = (int)Math.round(estimatedPose.getHeading().getValue());
        measurement[3] = (int)Math.round(towerAngle.getValue());
        for (int i = 4; i < 8; i++) {
            measurement[i] = (int)Math.round(lastIrMeasurement[i-4]);
        }
        return measurement;
    }

    /**
     * Rotate or move the robot a little distance towards the target position.
     * The robot will not move if the movement leads to a collision between
     * any of the features in "allFeatures". If the destination is reached
     * the robot will not move. The estimated pose is also updated and the 
     * value of "noise" is added to the estimate.
     * @param allFeatures ArrayList<Feature>
     * @param noise double
     */
    boolean moveRobot(double noise) {
        synchronized(movementLock) {
            if (!rotationFinished) {
                if (Math.abs(measuredRotation) >= Math.abs(targetRotation)){
                    measuredRotation = 0;
                    rotationFinished = true;
                }
                pose.rotate(new Angle(turnSpeed*rotationDirection));
                estimatedPose.rotate(new Angle((turnSpeed+noise)*rotationDirection));
                measuredRotation += (turnSpeed+noise)*rotationDirection;
            } else if (!translationFinished) {
                if(Math.abs(measuredDistance) >= Math.abs(targetDistance)) {
                    measuredDistance = 0;
                    translationFinished = true;
                    return true;
                }
                Pose testPose = Pose.copy(pose);
                testPose.move(moveSpeed*movementDirection);
                estimatedPose.move((moveSpeed+noise)*movementDirection);
                measuredDistance += (moveSpeed+noise)*movementDirection;
                if (world.checkIfPositionIsFree(testPose.getPosition(), id)) {
                    pose.move(moveSpeed*movementDirection);
                }
            }
            return false;
        }
    }
    
    /**
     * Compute the intersection between each sensors line of sight and any of
     * the features, and add the distance to the intersection as a new
     * measurement. The value given in "noise" is added to the measurement
     * @param features ArrayList<Feature>
     * @param sensorNoise double
     */
    void measureIR(double sensorNoise) {
        Position lineOfSightStart = pose.getPosition();
        Angle lineOfSightAngle = Angle.sum(towerAngle, pose.getHeading());
        for (int i = 0; i < 4; i++) {
            lastIrMeasurement[i] = 0;
            
            // Create a feature along the line of sight for each sensor
            if(i > 0){
                lineOfSightAngle.add(90);
            }
            Position offset = Utilities.polar2cart(lineOfSightAngle, maxVisualLength);
            Position lineOfSightEnd = Position.sum(lineOfSightStart, offset);
            Line lineOfSight = Line.convertFeatureToLine(new Feature(lineOfSightStart, lineOfSightEnd));
            lastIrMeasurement[i] = world.findNearestIntersection(lineOfSight, id);
            if(lastIrMeasurement[i] != 0){
                lastIrMeasurement[i] += sensorNoise;
            }
        }
    }
    
    
    static String generateHandshake(int robotWidth, int robotLength, int[] toweroffset, int axleoffset, int[] sensoroffset, int[] irheading, int messageDeadline) {
        // 02 is handshake
        return "{H," + robotWidth + "," + robotLength + "," + toweroffset[0] + "," + toweroffset[1] + "," + axleoffset + "," + sensoroffset[0] + "," + sensoroffset[1] + "," + sensoroffset[2] + "," + sensoroffset[3] + "," + irheading[0] + "," + irheading[1] + "," + irheading[2] + "," + irheading[3] + "," + messageDeadline + "}\n";
    }

    static String generateUpdate(int xPos, int yPos, int robotHeading, int towerHeading, int s1, int s2, int s3, int s4) {
        // 01 is update
        return "{U," + xPos + "," + yPos + "," + robotHeading + "," + towerHeading + "," + s1 + "," + s2 + "," + s3 + "," + s4 + "}\n";
    }
    
    String getHandShakeMessage() {
        int robotWidth = 40;
        int robotLength = 60;
        int toweroffset[] = {30, 40};
        int axleoffset = 0;
        int[] sensoroffset = {5, 5, 5, 5};
        int[] irheading = {0, 90, 180, 270};
        int messageDeadline = 400;
        String robotHandshake = generateHandshake(robotWidth, robotLength, toweroffset, axleoffset, sensoroffset, irheading, messageDeadline);
        return robotHandshake;
    }
    
}
