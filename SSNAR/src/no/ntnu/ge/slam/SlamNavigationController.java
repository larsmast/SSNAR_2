/**
 * This code is written as part of a Master Thesis
 * the spring of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 */
package no.ntnu.ge.slam;

import no.ntnu.et.general.Angle;
import static no.ntnu.et.map.MapLocation.getOctant;
import no.ntnu.et.simulator.SlamRobot;

/**
 *
 * @author geirhei
 */
public class SlamNavigationController extends Thread {
    private boolean paused = false;
    SlamRobot robot;
    int[] command;
    WindowMap localWindow;
    boolean collision = false;
    int[][] windowArray;
    
    public SlamNavigationController(SlamRobot robot) {
        this.robot = robot;
        command = new int[] {0, 0};
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
        
        robot.setTarget(0, 100);
        robot.setBusy(true);
        
        while (true) {
            try {
                Thread.sleep(5000);
            } catch (InterruptedException e) {
                e.printStackTrace();
                break;
            }
            if (paused) {
                continue;
            }
            
            if (!robot.isBusy()) {
                //int currentOrientation = robot.getRobotOrientation();
                //int newOrientation = getNewOrientation(currentOrientation, 1);
                //int rotation = newOrientation - currentOrientation;
                int distance = localWindow.getHeight();
                //robot.setTarget(rotation, distance);
                robot.setTarget(0, distance);
                robot.setBusy(true);
            }
            
            int frontLine = robot.getRobotWindowLocation().getRow();
            int column = robot.getRobotWindowLocation().getColumn();
            collision = false;
            for (int i = 0; i < 6; i++) {
                for (int j = -2; j < 2; j++) {
                    if (windowArray[frontLine+15+i][column+j] == 1) {
                        collision = true;
                        robot.setTarget(0, 0);
                        robot.setBusy(false);
                        int currentOrientation = robot.getRobotOrientation();
                        int newOrientation = getNewOrientation(currentOrientation, 1);
                        int rotation = newOrientation - currentOrientation;
                        int distance = localWindow.getHeight();
                        robot.setTarget(rotation, distance);
                        robot.setBusy(true);
                        break;
                    }
                }
                
            }
            
            // while (!origo of remoteWindow reached)
            
                // if (wall reached)
                    // status = not busy
                    //turn in some direction
                    // shift localWindow to current location
                    // get remoteWindow for the next direction

                // else if (fully explored ahead)
                    // status = not busy
                    // turn in some direction
                    // shift localWindow to current location
                    // get remoteWindow for the next direction

                // else if (unexplored fields ahead, middle of remoteWindow reached)
                    // status = busy
                    // don't change direction
                    // localWindow = remoteWindow
                    // get remoteWindow for area ahead
                
            /*
            if (!robot.isBusy() && !robot.isInCollisionManagement()) {
                robot.setTarget(90, 100);
                robot.setBusy(true);
            }
            */
        }
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