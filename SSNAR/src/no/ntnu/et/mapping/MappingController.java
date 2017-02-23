/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.mapping;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import javafx.collections.ObservableList;
import no.ntnu.tem.application.RobotController;
import no.ntnu.et.map.GridMap;
import no.ntnu.et.map.MapLocation;
import no.ntnu.et.general.Angle;
import no.ntnu.et.general.Pose;
import no.ntnu.et.general.Position;
import no.ntnu.et.map.Cell;
import no.ntnu.tem.robot.Robot;

/**
 * This class creates the map from the measurements. See (Thon 2016) for more
 * information on how it works.
 * 
 * @author Eirik Thon
 */
public class MappingController extends Thread {
    ArrayList<String> robotNames;
    private GridMap map;
    private RobotController robotController;
    private HashMap<String, MeasurementHandler> measurementHandlers;
    private Object nameLock = new Object();
    private boolean paused;
    private Thread mapCleaner;
    
    private final boolean debug = true;

    /**
     * Constructor
     * @param rc
     * @param map 
     */
    public MappingController(RobotController rc, GridMap map) {
        measurementHandlers = new HashMap<String, MeasurementHandler>();
        robotNames = new ArrayList<String>();
        robotController = rc;
        this.map = map;
        setName("Mapping");
        mapCleaner = new Thread(new MapCleaningWorker());
        mapCleaner.start();
        mapCleaner.setName("Map Cleaner");
    }
    
    /**
     * Adds a new robot to the mapping process. The mapping controller will
     * start to add measurements from the new robot into the map
     * @param name 
     */
    public void addRobot(String name){
        robotNames.add(name);
        int[] initialRobotPose = robotController.getRobot(name).getInitialPosition();
        Pose initialPose = new Pose(initialRobotPose[0], initialRobotPose[1], initialRobotPose[2]);
        MeasurementHandler newHandler = new MeasurementHandler(robotController.getRobot(name), initialPose);
        measurementHandlers.put(name, newHandler);
        int[] initialPosition = {(int)Math.round(initialPose.getPosition().getXValue()), (int)Math.round(initialPose.getPosition().getYValue())};
        robotController.getRobot(name).setPosition(initialPosition);
        robotController.getRobot(name).setRobotOrientation((int)Math.round(initialPose.getHeading().getValue()));
        robotController.getRobot(name).setDestination(initialPosition);
        map.resize(initialPose.getPosition());
    }
    
    /**
     * Removes a robot from the mapping process
     * @param name 
     */
    public void removeRobot(String name){
        robotNames.remove(name);
        measurementHandlers.remove(name);
    }
    
    /**
     * Starts the mapping.
     */
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
    
    
    /**
     * This is the core of the mapping process. The method updates the
     * measurement handlers for each robot and adds the measurements into the
     * map. Line of sight is also added into the map.
     */
    @Override
    public void run() {
        
        // For testing
        int maxFrontierLocations = 0;
        int maxOccupied = 0;
        
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
            for (int i = 0; i < robotNames.size(); i++) {
                String name = robotNames.get(i);
                Robot robot = robotController.getRobot(name);
                if (measurementHandlers.get(name).updateMeasurement() == false) {
                    continue;
                }

                Position robotPosition = measurementHandlers.get(name).getRobotPosition();
                Angle robotAngle = measurementHandlers.get(name).getRobotHeading();

                int[]position = {(int)Math.round(robotPosition.getXValue()), (int)Math.round(robotPosition.getYValue())};
                robot.setPosition(position);
                robot.setRobotOrientation((int)Math.round(robotAngle.getValue()));

                // Find the location of the robot in the map
                map.resize(robotPosition);
                MapLocation robotLocation = map.findLocationInMap(robotPosition);

                Sensor[] sensors = measurementHandlers.get(name).getIRSensorData();
                for(Sensor sensor: sensors){
                    boolean tooClose = false;
                    
                    // Check the distance between the position of the measurement and all the other robots
                    for(int j = 0; j < robotNames.size(); j++){
                        String otherName = robotNames.get(j);
                        int[] otherPositionInt = robotController.getRobot(otherName).getPosition();
                        Position otherPosition = new Position(otherPositionInt[0], otherPositionInt[1]);
                        if(Position.distanceBetween(otherPosition, sensor.getPosition()) < 10){
                            tooClose = true;
                            break;
                        }
                    }
                    
                    // The measurement is only added to the map if it is at a certain distance to the other robots
                    if(!tooClose){
                        map.resize(sensor.getPosition());
                        MapLocation measurementLocation = map.findLocationInMap(sensor.getPosition());
                        if(sensor.isMeasurement()){
                            map.addMeasurement(measurementLocation, true);
                        }
                        
                        // Create a measurements indicating no obstacle in the sensors line of sight
                        ArrayList<MapLocation> lineOfSight = getLineBetweenPoints(robotLocation, measurementLocation);
                        for (MapLocation location : lineOfSight) {
                            map.addMeasurement(location, false);
                        }
                    }
                }
            }
            
            /*
            if (debug) {
                int frontierLocations = map.getFrontierLocations().size();
                if (frontierLocations > maxFrontierLocations) {
                    maxFrontierLocations = frontierLocations;
                }
                System.out.println("Max frontier locations: " + maxFrontierLocations);
                
                int cellCount = 0;
                for (int i = map.getBottomRow(); i <= map.getTopRow(); i++) {
                    for (int j = map.getLeftColumn(); j <= map.getRightColumn(); j++) {
                        MapLocation location = new MapLocation(i, j);
                        Cell cell = map.findCell(location);
                        if (cell.isOccupied()) {
                            cellCount++;
                        }
                    }
                }
                if (cellCount > maxOccupied) {
                    maxOccupied = cellCount;
                }
                System.out.println("Max occupied locations: " + maxOccupied);
            }
            */
        }
    }
    
    /**
     * Returns all map locations in a straight line between two MapLocations.
     * Uses Bresenham's line algorithm
     * @param loc1
     * @param loc2
     * @return 
     */
    public static ArrayList<MapLocation> getLineBetweenPoints(MapLocation loc1, MapLocation loc2){
        int dx = loc2.getColumn() - loc1.getColumn();
        int dy = loc2.getRow() - loc1.getRow();
        double angle = Math.toDegrees(Math.atan2(dy, dx));
        if(angle < 0){
            angle += 360;
        }
        int oct = MapLocation.getOctant(angle);
        MapLocation locOct = MapLocation.switchToOctantZeroFrom(oct, new MapLocation(dy, dx));
        ArrayList<MapLocation> lineOct = MappingController.bresenham(new MapLocation(0,0), locOct);
        ArrayList<MapLocation> line = new ArrayList<MapLocation>();
        for(MapLocation loc : lineOct){
            line.add(MapLocation.sum(loc1, MapLocation.switchFromOctantZeroTo(oct, loc)));
        }
        return line;
    }
    
    /**
     * Bresenham's line algorithm. Found on
     * https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
     * @param start
     * @param end
     * @return 
     */
    static public ArrayList<MapLocation> bresenham(MapLocation start, MapLocation end) {
        ArrayList<MapLocation> ray = new ArrayList<MapLocation>();
        int startX = start.getColumn();
        int startY = start.getRow();
        int endX = end.getColumn();
        int endY = end.getRow();
        ray.add(new MapLocation(startY, startX));

        int dx = endX - startX;
        int dy = endY - startY;
        int D = 2 * dy - dx;
        int y = startY;
        if (D > 0) {
            y = y + 1;
            D = D - (2 * dx);
        }
        for (int x = startX + 1; x < endX; x++) {
            ray.add(new MapLocation(y, x));
            D = D + (2 * dy);
            if (D > 0) {
                y = y + 1;
                D = D - (2 * dx);
            }
        }
        return ray;
    }
    

    /**
     * Worker thread used for filling in unexplored gaps in the map.
     */
    private class MapCleaningWorker implements Runnable{
        
        public MapCleaningWorker() {

        }
        
        @Override
        public void run() {
            int cleanUpCountDown = 0;
            while (true) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                    break;
                }
                cleanUpCountDown++;
                if(cleanUpCountDown == 100){
                    cleanUpCountDown = 0;
                    map.cleanUp();
                }
            }
        }
    }
}
