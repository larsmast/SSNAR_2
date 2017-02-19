/**
 * This code is written as part of a Master Thesis
 * the spring of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 */
package no.ntnu.ge.slam;

import java.util.ArrayList;
import java.util.concurrent.LinkedBlockingQueue;
import no.ntnu.et.general.Angle;
import no.ntnu.et.general.Pose;
import no.ntnu.et.general.Position;
import no.ntnu.et.map.MapLocation;
import static no.ntnu.et.map.MapLocation.getOctant;
import static no.ntnu.et.map.MapLocation.sum;
import static no.ntnu.et.mapping.MappingController.getLineBetweenPoints;
import no.ntnu.et.mapping.Sensor;
import no.ntnu.et.simulator.SlamRobot;
import no.ntnu.tem.communication.Inbox;

/**
 *
 * @author geirhei
 */
public class SlamMappingController extends Thread {
    private final int cellSize = 2;
    private SlamRobot robot;
    private Inbox inbox;
    private WindowMap localWindow;
    private boolean paused;
    private LinkedBlockingQueue<int[]> updateQueue;
    private SlamMeasurementHandler measurementHandler;
    private MapLocation origoLocation;
    
    public SlamMappingController(SlamRobot robot, Inbox inbox) {
        this.robot = robot;
        this.inbox = inbox;
        localWindow = robot.getWindowMap();
        updateQueue = robot.getUpdateQueue();
        measurementHandler = new SlamMeasurementHandler(robot);
        origoLocation = null;
        
    }
    
    @Override
    public void start(){
        if(!isAlive()){
            super.start();
        }
        paused = false;
    }

    /**
     * Pauses the mapping
     */
    public void pause(){
        paused = true;
    }
    
    /**
     * Returns if the mapping is running or paused
     * @return 
     */
    public boolean isRunning(){
        return !paused;
    }
    
    @Override
    public void run() {
        //fillTestRemoteWindow(); // For testing with simple occupied row in window
        /*
        remoteWindow.setGlobalStartRow(remoteWindow.getHeight());
        remoteWindow.setGlobalStartColumn(0); // already set
        */
        
        int[][] testWindow = localWindow.getWindow();
        
        while (true) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
                break;
            }
            if (paused) {
                continue;
            }
            
            if (measurementHandler.updateMeasurement() == false) {
                continue;
            }
            
            Position robotPosition = measurementHandler.getRobotPosition();
            Angle robotAngle = measurementHandler.getRobotHeading();
            //Pose robotPose = new Pose(robotPosition, robotAngle);
            
            robot.setRobotOrientation((int) robotAngle.getValue());

            // Find the location of the robot in the global map
            MapLocation robotGlobalLocation = findLocationInGlobalMap(robotPosition);
            //System.out.println("robotGlobalLocation: Row: " + robotGlobalLocation.getRow() + ", Column: " + robotGlobalLocation.getColumn());
            // Check if robot is busy, if not set origoLocation to current location
            if (origoLocation == null || !robot.isBusy()) {
                origoLocation = robotGlobalLocation;
                //System.out.println("origoLocation: Row: " + origoLocation.getRow() + ", Column: " + origoLocation.getColumn());
            }
            // Find the location of the robot in the window
            MapLocation robotWindowLocation = findLocationInWindow(robotAngle, robotGlobalLocation);
            robot.setRobotWindowLocation(robotWindowLocation);
            System.out.println("robotWindowLocation: Row: " + robotWindowLocation.getRow() + ", Column: " + robotWindowLocation.getColumn());
            Sensor[] sensors = measurementHandler.getIRSensorData();
            for (Sensor sensor : sensors) {
                //boolean tooClose = false; - does not care about position of other robots
                MapLocation globalMeasurementLocation = findLocationInGlobalMap(sensor.getPosition());
                MapLocation windowMeasurementLocation = findLocationInWindow(robotAngle, globalMeasurementLocation);
                if (sensor.isMeasurement()) {
                    localWindow.addMeasurement(windowMeasurementLocation, true);
                }
                
                // Create a measurements indicating no obstacle in the sensors line of sight
                ArrayList<MapLocation> lineOfSight = getLineBetweenPoints(robotWindowLocation, windowMeasurementLocation);
                for (MapLocation location : lineOfSight) {
                    localWindow.addMeasurement(location, false);
                }
            //map.print();
            }
            localWindow.print();
            //localWindow.testPrint();

        }
    }
    
    /**
     * Returns the MapLocation that corresponds to the specified position.
     * From GridMap.java by Eirik Thon.
     * 
     * @param position
     * @return MapLocation
     */
    private MapLocation findLocationInGlobalMap(Position position) {
        int row = 0;
        if (position.getYValue() >= 0) {
            row = (int)(position.getYValue()/cellSize);
        } else {
            if (position.getYValue()%cellSize == 0) {
                row = (int)(position.getYValue()/cellSize);
            } else {
                row = (int)(position.getYValue()/cellSize)-1;
            }
        }
        int column = 0;
        if (position.getXValue() >= 0) {
            column = (int)(position.getXValue()/cellSize);
        } else {
            if (position.getXValue()%cellSize == 0) {
                column = (int)(position.getXValue()/cellSize);
            } else {
                column = (int)(position.getXValue()/cellSize)-1;
            }
        }
        return new MapLocation(row, column);
    }
    
    private MapLocation findLocationInWindow(Angle robotAngle, MapLocation globalMapLocation) {
        int dx = globalMapLocation.getColumn() - origoLocation.getColumn();
        int dy = globalMapLocation.getRow() - origoLocation.getRow();
        //System.out.println("dx: " + dx + ", dy: " + dy);
        
        MapLocation offset;
        MapLocation startLocation = new MapLocation(24, 24);
        MapLocation windowLocation;
        
        double heading = robotAngle.getValue(); // print value of heading?
        int octant = getOctant(heading);
        switch (octant) {
            case 0:
            case 7:
                offset = new MapLocation(dx, dy);
                break;
            case 1:
            case 2:
                offset = new MapLocation(dy, dx);
                break;
            case 3:
            case 4:
                offset = new MapLocation(-dx, dy);
                break;
            case 5:
            case 6:
                offset = new MapLocation(-dy, -dx);
                break;
            default:
                System.out.println("Invalid octant!");
                offset = new MapLocation(0, 0);
                break;
        }
        
        windowLocation = sum(startLocation, offset);
        //System.out.println("Row: " + windowLocation.getRow() + ", Col: " + windowLocation.getColumn());
        return windowLocation;
    }
    
    // fix: negative locations
    /*
    private boolean robotHasMoved(MapLocation currentLoc, MapLocation newLoc) {
        int dx = newLoc.getRow() - currentLoc.getRow();
        int dy = newLoc.getColumn() - currentLoc.getColumn();
        return !(dx == 0 && dy == 0);
    }
    */
    /*
    private void fillTestRemoteWindow() {
        int[][] window = remoteWindow.getWindow();
        for (int i = 0; i < remoteWindow.getHeight(); i++) {
            for (int j = 0; j < remoteWindow.getWidth(); j++) {
                window[i][j] = 2;
            }
        }
        for (int k = 0; k < remoteWindow.getWidth(); k++) {
            window[75][k] = 1;
        }
    }
    */
    
}
