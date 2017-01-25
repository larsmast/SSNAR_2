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
import no.ntnu.et.general.Position;
import no.ntnu.et.map.Cell;
import no.ntnu.et.map.MapLocation;
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
    private int[][] mapWindow;
    private boolean paused;
    private LinkedBlockingQueue<int[]> updateQueue;
    //private int[] currentUpdate;
    private SlamMeasurementHandler measurementHandler;
    private MapLocation previousLocation;
    
    public SlamMappingController(SlamRobot robot, Inbox inbox) {
        this.robot = robot;
        this.inbox = inbox;
        mapWindow = robot.getMapWindow();
        updateQueue = robot.getUpdateQueue();
        //currentUpdate = new int[20];
        measurementHandler = new SlamMeasurementHandler(robot);
        previousLocation = this.robot.getInitialLocation();
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
            if (paused){
                continue;
            }
            
            if (measurementHandler.updateMeasurement() == false) {
                continue;
            }
            
            Position robotPosition = measurementHandler.getRobotPosition();
            //Angle robotAngle = measurementHandler.getRobotHeading();

            // Find the location of the robot in the map
            MapLocation robotLocation = findLocationInMap(robotPosition);
            
            // if changed: resize
            
            
            Sensor[] sensors = measurementHandler.getIRSensorData();
            for (Sensor sensor : sensors) {
                //boolean tooClose = false; - does not care about position of other robots
                
                //resize?
                MapLocation measurementLocation = findLocationInMap(sensor.getPosition());
                if(sensor.isMeasurement()){
                    addMeasurement(measurementLocation, true);
                }
                
                // Create a measurements indicating no obstacle in the sensors line of sight
                ArrayList<MapLocation> lineOfSight = getLineBetweenPoints(robotLocation, measurementLocation);
                for (MapLocation location : lineOfSight) {
                    addMeasurement(location, false);
                }
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
    private MapLocation findLocationInMap(Position position) {
        int row = 0;
        if(position.getYValue() >= 0){
            row = (int)(position.getYValue()/cellSize);
        }else{
            if(position.getYValue()%cellSize == 0){
                row = (int)(position.getYValue()/cellSize);
            }else{
                row = (int)(position.getYValue()/cellSize)-1;
            }
        }
        int column = 0;
        if(position.getXValue() >= 0){
            column = (int)(position.getXValue()/cellSize);
        }else{
            if(position.getXValue()%cellSize == 0){
                column = (int)(position.getXValue()/cellSize);
            }else{
                column = (int)(position.getXValue()/cellSize)-1;
            }
        }
        return new MapLocation(row, column);
    }
    
    /**
     * Updates the map.
     * (This method also updates the restricted and weakly
     * restricted area of the map if the occupied status of a cell changes.)
     * 
     * @param location
     * @param measurement 
     */
    private void addMeasurement(MapLocation location, boolean measurement) {
        int row = location.getRow();
        int col = location.getColumn();
        if (measurement) {
            mapWindow[row][col] = 1; //occupied
        } else {
            mapWindow[row][col] = 0; // free
        } // (unexplored = 2)

        // If the cell changes from occupied to free or vice versa, the restricted
        // status of nearby cells are updated here:
        /*
        if(measuredCell.stateChanged()){
            ArrayList<MapLocation> restricted = createCircle(location, 15);
            ArrayList<MapLocation> weaklyRestricted = createCircle(location, 25);
            for(MapLocation location2: restricted){
                if(measuredCell.isOccupied()){
                    map.get(location2).addRestrictingCell(measuredCell);
                }
                else {
                    map.get(location2).removeRestrictingCell(measuredCell);
                }
            }
            for(MapLocation location2: weaklyRestricted){
                if(measuredCell.isOccupied()){
                    map.get(location2).addWeaklyRestrictingCell(measuredCell);
                }
                else {
                    map.get(location2).removeWeaklyRestrictingCell(measuredCell);
                }
            }
        }
        */
    }
}
