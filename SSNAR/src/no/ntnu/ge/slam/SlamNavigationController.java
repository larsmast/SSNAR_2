/**
 * This code is written as part of a Master Thesis
 * the spring of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 */
package no.ntnu.ge.slam;

import static no.ntnu.et.map.MapLocation.getOctant;
import no.ntnu.et.simulator.SlamRobot;

/**
 *
 * @author geirhei
 */
public class SlamNavigationController extends Thread {
    private boolean paused = false;
    private boolean debug = true;
    private SlamRobot robot;
    //int[] command;
    private WindowMap localMap;
    private boolean collision = false;
    private int[][] windowArray;
    private int frontDistance;
    private int leftDistance;
    private int rightDistance;
    private final int cellSize = 2;
    private final int distanceLimit = 10; //map cells
    private final int sideDistanceLimit = distanceLimit;
    
    private enum Direction {
        FORWARD, LEFT, RIGHT, BACKWARDS
    }
    
    public SlamNavigationController(SlamRobot robot) {
        this.robot = robot;
        //command = new int[] {0, 0};
        localMap = robot.getWindowMap();
        windowArray = localMap.getWindow();
        frontDistance = robot.getLineOfSight();
        leftDistance = frontDistance;
        rightDistance = frontDistance;
    }
    
    @Override
    public void start() {
        if (!isAlive()) {
            super.start();
        }
        paused = false;
    }
    
    public void pause() {
        paused = true;
    }
    
    @Override
    public void run() {
        setName("Slam navigation controller");
        moveStop();
        while (true) {
            try {
                Thread.sleep(100); // was 5000
            } catch (InterruptedException e) {
                e.printStackTrace();
                break;
            }
            if (paused) {
                continue;
            }
            
            if (!robot.isBusy()) {
                moveForward();
            }
            checkSurroundings();
            if (leftDistance < sideDistanceLimit) {
                turnRight();
                while (!robot.isRotationFinished()) {
                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {}
                }
            }
            if (rightDistance < sideDistanceLimit) {
                turnLeft();
                while (!robot.isRotationFinished()) {
                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {}
                }
            }
            /*
            Include more functionality, include decideDirection()
            Should there be a limit to how far the robot drives before it
            needs to make a new decision?
            */
        }
    }
    
    private Direction decideDirection() {
        checkSurroundings();
        if (leftDistance > rightDistance && leftDistance > frontDistance) {
            if (debug) {
                System.out.println("Direction decision: LEFT");
            }
            return Direction.LEFT;
        } else if (rightDistance > leftDistance && rightDistance > frontDistance) {
            if (debug) {
                System.out.println("Direction decision: RIGHT");
            }
            return Direction.RIGHT;
        } else if (leftDistance < sideDistanceLimit && rightDistance < sideDistanceLimit && frontDistance < distanceLimit) {
            if (debug) {
                System.out.println("Direction decision: BACKWARDS");
            }
            return Direction.BACKWARDS;
        } else {
            if (debug) {
                System.out.println("Direction decision: FORWARD");
            }
            return Direction.FORWARD;
        }
    }
    
    private void checkSurroundings() {
        frontDistance = getFrontDistance();
        if (frontDistance < distanceLimit) {
            moveStop();
        }
        leftDistance = getLeftDistance();
        if (leftDistance < sideDistanceLimit) {
            moveStop();
        }
        rightDistance = getRightDistance();
        if (rightDistance < sideDistanceLimit) {
            moveStop();
        }
        if (debug) {
            System.out.println("Check surroundings done");
        }
    }
    
    private int getFrontDistance() {
        int robotColumn = robot.getRobotWindowLocation().getColumn();
        int robotRow = robot.getRobotWindowLocation().getRow();
        for (int i = 1; i < robot.getLineOfSight()/cellSize+1; i++) {
            if (localMap.getWindow()[robotRow+i][robotColumn] == 1) {
                return i;
            }
        }
        return -1;
    }
    
    private int getLeftDistance() {
        int robotColumn = robot.getRobotWindowLocation().getColumn();
        int robotRow = robot.getRobotWindowLocation().getRow();
        for (int i = 1; i < robot.getLineOfSight()/cellSize+1; i++) {
            if (localMap.getWindow()[robotRow][robotColumn-i] == 1) {
                return i;
            }
        }
        return -1;
    }
    
    private int getRightDistance() {
        int robotColumn = robot.getRobotWindowLocation().getColumn();
        int robotRow = robot.getRobotWindowLocation().getRow();
        for (int i = 1; i < robot.getLineOfSight()/cellSize+1; i++) {
            if (localMap.getWindow()[robotRow][robotColumn+i] == 1) {
                return i;
            }
        }
        return -1;
    }
    
    private void moveForward() {
        // move forward until stopped elsewhere
        robot.setTarget(0, 1000);
        robot.setBusy(true);
        if (debug) {
            System.out.println("Moving forward");
        }
    }
    
    private void turnLeft() {
        robot.setTarget(90, 0);
        robot.setBusy(true);
        if (debug) {
            System.out.println("Turning left");
        }
    }
    
    private void turnRight() {
        robot.setTarget(-90, 0);
        robot.setBusy(true);
        if (debug) {
            System.out.println("Turning right");
        }
    }
    
    private void moveStop() {
        robot.setTarget(0, 0);
        robot.setBusy(false);
        if (debug) {
            System.out.println("Stopping");
        }
    }
    
    private int getNewDistance() {
        int targetRow = localMap.getHeight() - localMap.getHeight()/4 - 1;
        int distance = targetRow - robot.getRobotWindowLocation().getRow();
        return distance;
    }
    
    private boolean checkCollision() {
        int robotRow = robot.getRobotWindowLocation().getRow();
        int robotColumn = robot.getRobotWindowLocation().getColumn();
        int safeDistance = 10;
        int frontRow = robotRow + safeDistance;
        for (int i = 1; i < safeDistance+1; i++) {
            for (int j = -safeDistance; j < safeDistance+1; j++) {
                try {
                    if (windowArray[robotRow+i][robotColumn+j] == 1) {
                        return true;
                    }
                } catch (ArrayIndexOutOfBoundsException e) {
                    e.printStackTrace();
                    System.out.println("ArrayIndexOutOfBounds in checkCollision()");
                }
                
            }
        }
        return false;
    }
    
    
    private void changeDirection() {
        
    }
    
    private int[] findCommandToTarget(int rotation, int distance) {
        int[] cmd = {rotation, distance};
        return cmd;
    }
    
    private int correctOrientation(int orientation) {
        int octant = getOctant(orientation);
        switch (octant) {
            case 0:
            case 7:
                return 0;
            case 1:
            case 2:
                return 90;
            case 3:
            case 4:
                return 180;
            case 5:
            case 6:
                return 270;
            default:
                System.out.println("Invalid octant!");
                return -1;
        }
    }
    
    private int getNewOrientation(int currentOrientation, int turnDirection) {
        if (currentOrientation == 270 && turnDirection == 1) {
            return 0;
        } else {
            return (currentOrientation + 90*turnDirection);
        }
    }
    
    private int updateMapOrientation(int turnAngle) {
        int currentOrientation = localMap.getOrientation();
        int newOrientation = currentOrientation + turnAngle;
        if (newOrientation < 0) {
            newOrientation += 360;
        } else if (newOrientation >= 360) {
            newOrientation -= 360;
        }
        return newOrientation;
    }

}