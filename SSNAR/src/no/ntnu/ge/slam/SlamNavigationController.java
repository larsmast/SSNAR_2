/**
 * This code is written as part of a Master Thesis
 * the spring of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 */
package no.ntnu.ge.slam;

import static java.lang.Math.sqrt;
import java.util.ArrayList;
import no.ntnu.et.general.Angle;
import no.ntnu.et.general.Position;
import no.ntnu.et.general.Utilities;
import no.ntnu.et.map.MapLocation;
import static no.ntnu.et.mapping.MappingController.getLineBetweenPoints;
import no.ntnu.et.simulator.SlamRobot;

/**
 *
 * @author geirhei
 */
public class SlamNavigationController extends Thread {
    private boolean paused = false;
    private boolean debug = true;
    private SlamRobot robot;
    private LocalMap localMap;
    private int frontDistance;
    private int leftDistance;
    private int rightDistance;
    private int leftDiagonalDistance;
    private int rightDiagonalDistance;
    private int[] distances;
    private final int cellSize = 2;
    private final int frontDistanceLimit = 20; //map cells
    private final int sideDistanceLimit = 10;
    
    private enum Direction {
        FORWARD, LEFT, RIGHT, BACKWARDS
    }
    
    public SlamNavigationController(SlamRobot robot) {
        this.robot = robot;
        localMap = robot.getWindowMap();
        frontDistance = robot.getLineOfSight()/cellSize;
        leftDistance = frontDistance;
        rightDistance = frontDistance;
        leftDiagonalDistance = frontDistance;
        rightDiagonalDistance = frontDistance;
        distances = new int[5];
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
            
            checkSurroundings();
            if (!robot.isBusy()) {
                Direction turnDirection = decideDirection();
                try {
                    switch (turnDirection) {
                        case LEFT:
                            turnLeft();
                            Thread.sleep(1500);
                            break;
                        case RIGHT:
                            turnRight();
                            Thread.sleep(1500);
                            break;
                        case BACKWARDS:
                            moveBackwards();
                            Thread.sleep(1500);
                            break;
                        case FORWARD:
                            moveForward();
                            break;
                    }
                } catch (InterruptedException e) {}
                
            }
            
        }
    }
    
    private Direction decideDirection() {
        if (frontDistance < frontDistanceLimit) {
            if (leftDiagonalDistance < frontDistanceLimit && leftDiagonalDistance < rightDiagonalDistance) {
                if (debug) {
                    System.out.println("Direction decision: RIGHT");
                }
                return Direction.RIGHT;
            } else if (rightDiagonalDistance < frontDistanceLimit && rightDiagonalDistance < leftDiagonalDistance) {
                if (debug) {
                    System.out.println("Direction decision: LEFT");
                }
                return Direction.LEFT;
            } else if (leftDistance < sideDistanceLimit && rightDistance < sideDistanceLimit) {
                if (debug) {
                    System.out.println("Direction decision: BACKWARDS");
                }
                return Direction.BACKWARDS;
            } else {
                if (debug) {
                    System.out.println("Direction decision: RIGHT (default)");
                }
                return Direction.RIGHT;
            }
        
        } else if (leftDiagonalDistance < frontDistanceLimit) {
            if (debug) {
                System.out.println("Direction decision: RIGHT");
            }
            return Direction.RIGHT;
        } else if (rightDiagonalDistance < frontDistanceLimit) {
            if (debug) {
                System.out.println("Direction decision: LEFT");
            }
            return Direction.LEFT;
            
        } else if (leftDistance < sideDistanceLimit && leftDiagonalDistance < sideDistanceLimit) {
            if (debug) {
                System.out.println("Direction decision: RIGHT");
            }
            return Direction.RIGHT;
            
        } else if (rightDistance < sideDistanceLimit && rightDiagonalDistance < sideDistanceLimit) {
            if (debug) {
                System.out.println("Direction decision: LEFT");
            }
            return Direction.LEFT;
            
        } else {
            if (debug) {
                System.out.println("Decide direction failed, default decision: FORWARD");
            }
            return Direction.FORWARD;
        }
    }
    
    private void checkSurroundings() {
        findDistances();
        if (frontDistance < frontDistanceLimit) {
            moveStop();
        }
        if (leftDiagonalDistance < frontDistanceLimit) {
            moveStop();
        }
        if (leftDistance < sideDistanceLimit) {
            moveStop();
        }
        if (rightDiagonalDistance < frontDistanceLimit) {
            moveStop();
        }
        if (rightDistance < sideDistanceLimit) {
            moveStop();
        }
        if (debug) {
            System.out.println("Check surroundings done");
        }
    }
    
    
    private int getFrontDistance() {
        for (int i = 1; i < robot.getLineOfSight()/cellSize+1; i++) {
            if (localMap.getWindow()[localMap.getCenterRow()+i][localMap.getCenterColumn()] == 1) {
                return i;
            }
        }
        return robot.getLineOfSight()/2;
    }
    
    
    private void findDistances() {
        Angle lineOfSightAngle = Angle.sum(robot.getPose().getHeading(), new Angle(-90));
        for (int i = 0; i < 5; i++) { //left, forward, right, + diagonals
            distances[i] = robot.getLineOfSight()/cellSize; // reset
            if (i > 0) {
                lineOfSightAngle.add(45);
            }
            Position offset = Utilities.polar2cart(lineOfSightAngle, robot.getLineOfSight());
            MapLocation mapOffset = new MapLocation((int) offset.getYValue()/cellSize, (int) offset.getXValue()/cellSize);
            MapLocation lineOfSightEnd = MapLocation.sum(robot.getLocalRobotLocation(), mapOffset);
            ArrayList<MapLocation> lineOfSight = getLineBetweenPoints(robot.getLocalRobotLocation(), lineOfSightEnd);
            for (int j = 0; j < lineOfSight.size(); j++) {
                MapLocation currentLocation = lineOfSight.get(j);
                int currentRow = currentLocation.getRow();
                int currentColumn = currentLocation.getColumn();
                if (localMap.getWindow()[currentRow][currentColumn] == 1) {
                    int distance = (int) MapLocation.distance(robot.getLocalRobotLocation(), currentLocation);
                    distances[i] = distance;
                    break;
                }   
            }
        }
        
        rightDistance = distances[0];
        rightDiagonalDistance = distances[1];
        frontDistance = distances[2];
        leftDiagonalDistance = distances[3];
        leftDistance = distances[4];
        
        if (debug) {
            System.out.println("Left: " + leftDistance + ", LeftDiag: " + leftDiagonalDistance +
            ", Front: " + frontDistance + ", RightDiag: " + rightDiagonalDistance +
                    ", Right: " + rightDistance);
            System.out.println("Find distances done");
        }
    }
    
    /*
    private int getFrontDistance() {
        MapLocation lineOfSightStart
        
        for (int i = 1; i < robot.getLineOfSight()/cellSize+1; i++) {
            if (localMap.getWindow()[localMap.getCenterRow()+i][localMap.getCenterColumn()] == 1) {
                return i;
            }
        }
        return robot.getLineOfSight()/2;
    }
*/
    
    private int getLeftDistance() {
        for (int i = 1; i < robot.getLineOfSight()/cellSize+1; i++) {
            if (localMap.getWindow()[localMap.getCenterRow()][localMap.getCenterColumn()-i] == 1) {
                return i;
            }
        }
        return robot.getLineOfSight()/2;
    }
    
    private int getRightDistance() {
        for (int i = 1; i < robot.getLineOfSight()/cellSize+1; i++) {
            if (localMap.getWindow()[localMap.getCenterRow()][localMap.getCenterColumn()+i] == 1) {
                return i;
            }
        }
        return robot.getLineOfSight()/2;
    }
    
    private int getLeftDiagonalDistance() {
        int horizontalCells = (int) (1/sqrt(2)*(robot.getLineOfSight()/cellSize));
        for (int i = 1; i < horizontalCells; i++) {
            if (localMap.getWindow()[localMap.getCenterRow()+i][localMap.getCenterColumn()-i] == 1) {
                return (int) (sqrt(2)*i);
            }
        }
        return robot.getLineOfSight()/2;
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
        robot.setTarget(45, 0);
        robot.setBusy(true);
        if (debug) {
            System.out.println("Turning left");
        }
    }
    
    private void turnRight() {
        robot.setTarget(-45, 0);
        robot.setBusy(true);
        if (debug) {
            System.out.println("Turning right");
        }
    }
    
    private void moveBackwards() {
        robot.setTarget(180, 1000);
        robot.setBusy(true);
        if (debug) {
            System.out.println("Moving backwards");
        }
    }
    
    private void moveStop() {
        robot.setTarget(0, 0);
        robot.setBusy(false);
        if (debug) {
            System.out.println("Stopping");
        }
        
    }
}