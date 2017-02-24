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
    private LocalMap localMap;
    private boolean paused;
    private LinkedBlockingQueue<int[]> updateQueue;
    private SlamMeasurementHandler measurementHandler;
    
    private final boolean debug = false;
    
    public SlamMappingController(SlamRobot robot, Inbox inbox) {
        this.robot = robot;
        this.inbox = inbox;
        localMap = robot.getWindowMap();
        updateQueue = robot.getUpdateQueue();
        measurementHandler = new SlamMeasurementHandler(robot);
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

            // Find the location of the robot in the global map
            Position robotPosition = measurementHandler.getRobotPosition();
            //Angle robotAngle = measurementHandler.getRobotHeading();
            MapLocation newGlobalRobotLocation = findLocationInGlobalMap(robotPosition);
            
            if (newGlobalRobotLocation != robot.getGlobalRobotLocation()) {
                localMap.shift(robot.getGlobalRobotLocation(), newGlobalRobotLocation);
                robot.setGlobalRobotLocation(newGlobalRobotLocation);
            }
            
            Sensor[] sensors = measurementHandler.getIRSensorData();
            for (Sensor sensor : sensors) {
                //boolean tooClose = false; - does not care about position of other robots
                MapLocation localMeasurementLocation = findLocationInLocalMap(sensor.getPosition());
                if (sensor.isMeasurement()) {
                    localMap.addMeasurement(localMeasurementLocation, true);
                }
                
                // Create a measurements indicating no obstacle in the sensors line of sight
                ArrayList<MapLocation> lineOfSight = getLineBetweenPoints(localMap.getCenterLocation(), localMeasurementLocation);
                for (MapLocation location : lineOfSight) {
                    localMap.addMeasurement(location, false);
                }
            }
            
            if (debug) {
                // Test
                //localWindow.addRobotWindowLocation(robotWindowLocation);
                localMap.print();
            }
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
    
    private MapLocation findLocationInLocalMap(Position position) {
        MapLocation globalMapLocation = findLocationInGlobalMap(position);
        int dx = globalMapLocation.getColumn() - robot.getGlobalRobotLocation().getColumn();
        int dy = globalMapLocation.getRow() - robot.getGlobalRobotLocation().getRow();
        //System.out.println("dx: " + dx + ", dy: " + dy);
        
        MapLocation offset = new MapLocation(dy, dx);
        MapLocation centerLocation = new MapLocation(localMap.getCenterRow(), localMap.getCenterColumn());
        MapLocation windowLocation;
        
        windowLocation = sum(centerLocation, offset);
        //System.out.println("Row: " + windowLocation.getRow() + ", Col: " + windowLocation.getColumn());
        return windowLocation;
    }
    
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
