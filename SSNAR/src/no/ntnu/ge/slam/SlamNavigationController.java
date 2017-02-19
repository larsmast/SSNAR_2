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
    SlamRobot robot;
    //int[] command;
    WindowMap localWindow;
    boolean collision = false;
    int[][] windowArray;
    
    public SlamNavigationController(SlamRobot robot) {
        this.robot = robot;
        //command = new int[] {0, 0};
        localWindow = robot.getWindowMap();
        windowArray = localWindow.getWindow();
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
            
            if (collision = checkCollision()) {
                robot.setBusy(false);
            }
            int targetRow = localWindow.getHeight() - localWindow.getHeight()/4 - 1;
            if (!robot.isBusy()) {
                int distance = targetRow - robot.getRobotWindowLocation().getRow();
                if (collision) {
                    robot.setTarget(90, distance);
                    localWindow.setOrientation(updateMapOrientation(90));
                    try {
                        Thread.sleep(5000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    collision = checkCollision();
                } else {
                    robot.setTarget(0, distance);
                }
                robot.setBusy(true);
            }
        }
    }
    
    private boolean checkCollision() {
        int robotRow = robot.getRobotWindowLocation().getRow();
        int robotColumn = robot.getRobotWindowLocation().getColumn();
        int safeDistance = 20;
        int frontRow = robotRow + safeDistance;
        for (int i = 1; i < safeDistance+1; i++) {
            for (int j = -safeDistance; j < safeDistance+1; j++) {
                try {
                    if (windowArray[robotRow+i][robotColumn+j] == 1) {
                        return true;
                    }
                } catch (ArrayIndexOutOfBoundsException e) {
                    e.printStackTrace();
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
        int currentOrientation = localWindow.getOrientation();
        int newOrientation = currentOrientation + turnAngle;
        if (newOrientation < 0) {
            newOrientation += 360;
        } else if (newOrientation >= 360) {
            newOrientation -= 360;
        }
        return newOrientation;
    }
    
    /*
    private int findRotation(int offset, int currentOrientation, int newOrientation) {
        int difference = newOrientation - currentOrientation;
        if (difference >= 0) {
            return difference - offset;
        } else if (difference < 0) {
            return difference + offset;
        }
    }
    */

}