/**
 * This code is written as part of a Master Thesis
 * the spring of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 * 
 * Based on SimRobot.java
 */
package no.ntnu.et.simulator;

import no.ntnu.et.general.Angle;
import no.ntnu.et.general.Pose;
import no.ntnu.et.general.Position;

/**
 *
 * @author geirhei
 */
public class SlamRobot {
    private final SimWorld world;
    private Pose pose;
    private Pose estimatedPose;
    final private Pose initialPose;
    private final String name;
    private final int id;
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
    private final int diameter = 10;
    
    private int[][] mapWindow;
    
    public SlamRobot(SimWorld world, Pose initialPose, String name, int id) {
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
        
        mapWindow = new int[100][100];
    }
}
