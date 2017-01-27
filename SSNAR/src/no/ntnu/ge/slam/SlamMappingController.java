/**
 * This code is written as part of a Master Thesis
 * the spring of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 */
package no.ntnu.ge.slam;

import java.util.ArrayList;
import java.util.concurrent.LinkedBlockingQueue;
import no.ntnu.et.general.Pose;
import no.ntnu.et.general.Position;
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
    private WindowMap map;
    private boolean paused;
    private LinkedBlockingQueue<int[]> updateQueue;
    private SlamMeasurementHandler measurementHandler;
    //private MapLocation previousLocation;
    private MapLocation origoLocation;
    
    public SlamMappingController(SlamRobot robot, Inbox inbox) {
        this.robot = robot;
        this.inbox = inbox;
        map = this.robot.getWindowMap();
        updateQueue = this.robot.getUpdateQueue();
        measurementHandler = new SlamMeasurementHandler(robot);
        //previousLocation = this.robot.getInitialLocation();
        origoLocation = this.robot.getInitialLocation();
        
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

            // Find the location of the robot in the world map
            MapLocation robotLocation = findLocationInMap(robotPosition);
            
            // Check if robot has moved, if so: shift window
            map.shift(origoLocation, robotLocation);
            origoLocation = robotLocation;
            
            Sensor[] sensors = measurementHandler.getIRSensorData();
            for (Sensor sensor : sensors) {
                //boolean tooClose = false; - does not care about position of other robots
                
                //resize?
                MapLocation measurementLocation = findLocationInWindowMap(sensor.getOffsetPosition());
                if(sensor.isMeasurement()){
                    map.addMeasurement(measurementLocation, true);
                }
                
                // Create a measurements indicating no obstacle in the sensors line of sight
                //ArrayList<MapLocation> lineOfSight = getLineBetweenPoints(robotLocation, measurementLocation);
                ArrayList<MapLocation> lineOfSight = getLineBetweenPoints(new MapLocation(0,0), measurementLocation);
                for (MapLocation location : lineOfSight) {
                    map.addMeasurement(location, false);
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
    
    
    private MapLocation findLocationInWindowMap(Position position) {
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
        // compensate for negative indices
        row += map.getHeight()/2 - 1;
        column += map.getWidth()/2 - 1;
        return new MapLocation(row, column);
    }
    
}
