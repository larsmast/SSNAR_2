/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.navigation;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.ConcurrentHashMap;
import no.ntnu.et.general.Position;
import no.ntnu.et.map.GridMap;
import no.ntnu.et.map.MapLocation;
import no.ntnu.et.mapping.MappingController;
import no.ntnu.tem.robot.Robot;

/**
 * This class is used to manage target selection for all robots. It keeps track
 * of the target points of all the robots, and the threads that are working on
 * finding new targets
 * 
 * @author Eirik Thon
 */

public class RobotTaskManager{

    
    private GridMap map;

    private ConcurrentHashMap<String, MapLocation> temporaryTargets;

    private ConcurrentHashMap<String, MapLocation> currentTargets;
    
    private HashMap<String, RobotTaskWorker> tasksInProgress;
    
    private int currentNumberOfWorkers;

    private boolean paused = true;
    
    final private boolean debug = false;
    
    final private int targetSpacing = 5; //[cm]

    public RobotTaskManager(GridMap map) {
        this.map = map;
        temporaryTargets = new ConcurrentHashMap<String, MapLocation>();
        currentTargets = new ConcurrentHashMap<String, MapLocation>();
        tasksInProgress = new HashMap<String, RobotTaskWorker>();
        currentNumberOfWorkers = 0;
    }
    
    
    public void updateCurrentTarget(String robotName, MapLocation currentLocation) {
        currentTargets.put(robotName, currentLocation);
    }
    
    public void createNewTask(Robot robot, NavigationRobot navRobot, String name){
        RobotTaskWorker worker = new RobotTaskWorker(robot, navRobot, name);
        currentNumberOfWorkers++;
        tasksInProgress.put(name, worker);
        Thread t = new Thread(worker);
        t.start();
        t.setName("Task-finder " + currentNumberOfWorkers);
    }
    
    boolean isWorkingOnTask(String name){
        return tasksInProgress.containsKey(name);
    }
    
    private class RobotTaskWorker implements Runnable{
        private NavigationRobot navRobot;
        private Robot robot;
        private String name;
        private boolean done;
        
        
        public RobotTaskWorker(Robot robot, NavigationRobot navRobot, String name) {
            this.robot = robot;
            this.navRobot = navRobot;
            this.name = name;
            done = false;
            Position robotPosition = new Position(robot.getPosition());
            MapLocation robotLocation = map.findLocationInMap(robotPosition);
            currentTargets.put(name, robotLocation);
        }
        
        @Override
        public void run() {
            ArrayList<MapLocation> frontierLocations = map.getFrontierLocations();
            ArrayList<MapLocation> possibleTargets = selectSpreadLocations(frontierLocations);
            int currentOrientation = robot.getRobotOrientation();
            boolean assigned = false;
            // Assign the robot to move to one of the frontier locations
            while (!assigned){
                Position robotPosition = new Position(robot.getPosition());
                MapLocation robotLocation = map.findLocationInMap(robotPosition);
                
                // Find the most optimal targetpoint in the map given the robots current location and the target points of the other robots
                MapLocation bestTarget = findBestTarget(currentOrientation, robotLocation, possibleTargets, name);
                
                if(bestTarget == null){
                    if(debug){
                        System.out.println(name + ": No targetpoint found");
                    }
                    break;
                }
                
                robotPosition = new Position(robot.getPosition());
                robotLocation = map.findLocationInMap(robotPosition);
                if(map.findCell(robotLocation).isRestricted()){
                    if(debug){
                        System.out.println(name + ": Robot in restricted position");
                    }
                    break;
                }
                // Search for a path between the robot and the best target point
                ArrayList<MapLocation> path = PathPlanningFunctions.findPath(map, bestTarget, robotLocation);

                // If no path to bestTarget is found remove bestTarget from possibleTargets
                if(path == null){
                    for(int i = 0; i < possibleTargets.size(); i++){
                        if(MapLocation.equals(bestTarget, possibleTargets.get(i))){
                            possibleTargets.remove(i);
                            break;
                        }
                    }
                    // If there are no targets left in possibleTargets give up and try agin next iteration
                    if(possibleTargets.size() == 0) {
                        if(debug){
                            System.out.println(name + ": No targetpoints reachable. Waiting");
                        }
                        break;
                    }
                    if(debug){
                        System.out.println(name + ": Target unreachable. Finding new target");
                    }
                }
                // If a path is found find waypoints along the way and set destination for the robot
                else {
                    //computeUtility(bestTarget, robotLocation, currentOrientation, name, true);
                    if(debug){
                        System.out.println(name + ": Path found");
                    }
                    ArrayList<Position> newWaypoints = PathPlanningFunctions.generateWaypoints(map, path);
                    if(newWaypoints.size() == 0) {
                        if(debug){
                            System.out.println(name + ": Unable to generate waypoints");
                        }
                        temporaryTargets.remove(name);
                        break;
                    }
                    currentTargets.put(name, bestTarget);
                    temporaryTargets.remove(name);
                    navRobot.addWaypoints(newWaypoints);
                    Position destinationPos = newWaypoints.get(newWaypoints.size()-1);
                    int[] destination = {(int)Math.round(destinationPos.getXValue()), (int)Math.round(destinationPos.getYValue())};
                    robot.setDestination(destination);
                    assigned = true;
                }
            }
            currentNumberOfWorkers--;
            
            // Remove this thread from the current working threads
            tasksInProgress.remove(name);
        }
    }
    
    ArrayList<MapLocation> selectSpreadLocations(ArrayList<MapLocation> frontierLocations) {
        ArrayList<MapLocation> spreadLocations = new ArrayList<MapLocation>();
        for(MapLocation target: frontierLocations){
            boolean done = false;
            for(MapLocation existingTarget: spreadLocations){
                double distance = MapLocation.distance(target, existingTarget)*map.getCellSize();
                if(distance < targetSpacing){
                    done = true;
                    break;
                }
            }
            if(done){
                continue;
            }
            else{
                spreadLocations.add(target);
            }
        }
        return spreadLocations;
    }
    
    MapLocation findBestTarget(int currentOrientation, MapLocation currentLocation, ArrayList<MapLocation> possibleTargetLocations, String robotName){
        MapLocation bestTargetPoint = null;
        double bestUtility = Double.NEGATIVE_INFINITY;
        for (MapLocation targetPoint: possibleTargetLocations) {
            // map.findCell(targetPoint).setTarget();
            double utility = computeUtility(targetPoint, currentLocation, currentOrientation, robotName, false);
            if(utility > bestUtility) {
                bestUtility = utility;
                bestTargetPoint = MapLocation.copy(targetPoint);
                temporaryTargets.put(robotName, bestTargetPoint);
            }
        }
        return bestTargetPoint;    
    }
    
    private double computeUtility(MapLocation target, MapLocation currentLocation, int currentOrientation, String robotName, boolean print){
        int mapCellSize = map.getCellSize();
        
        double exploration = map.countUnknownCellsAroundLocation(target, 30)*mapCellSize*mapCellSize;
        
        double turnDistance = MapLocation.angleBetween(currentLocation, target) - currentOrientation;
        turnDistance = Math.abs((turnDistance + 180) % 360 - 180);

        double distance = MapLocation.distance(target, currentLocation)*map.getCellSize();
        
        double tooNear = 0;
        if(distance < 5){
            tooNear = Double.POSITIVE_INFINITY;
        }
        
        double distribution = 0;
        for(ConcurrentHashMap.Entry<String, MapLocation> entry : temporaryTargets.entrySet()){
            if(entry.getKey() != robotName){
                if(MapLocation.distance(target, entry.getValue())*mapCellSize <= 50){
                    distribution = Double.POSITIVE_INFINITY;
                }else{
                    distribution += 1/MapLocation.distance(target, entry.getValue())*mapCellSize;
                }
            }
        }
        for(ConcurrentHashMap.Entry<String, MapLocation> entry : currentTargets.entrySet()){
            if(entry.getKey() != robotName){
                if(MapLocation.distance(target, entry.getValue())*mapCellSize <= 50){
                    distribution = Double.POSITIVE_INFINITY;
                }else{
                    distribution += 1/MapLocation.distance(target, entry.getValue())*mapCellSize;
                }
            }
        }
        distribution = distribution / (currentTargets.size()+temporaryTargets.size()+1);
        
        double lineOfSight = 1;
        ArrayList<MapLocation> shortestPath = MappingController.getLineBetweenPoints(currentLocation, target);
        for(MapLocation location: shortestPath){
            if(!map.findCell(location).isWeaklyTargetable()){
                lineOfSight = 0;
                break;
            }
        }
        
        double closeToWall = 0;
        if(map.findCell(target).isWeaklyRestricted()){
            closeToWall = 1;
        }
        
        double weight1 = 0.2;
        double weight2 = 3;
        double weight3 = 40000;
        double weight4 = 300;
        double weight5 = 2000;
        double weight6 = 1;
        //return exploration*0.5-distance*5*+distribution*1+lineOfSight*1;
        if(print){
            /*
            System.out.println("Base UTILITY:");
            System.out.println(exploration);
            System.out.println(distance);
            System.out.println(distribution);
            System.out.println(lineOfSight);
            System.out.println(closeToWall);
            */
            System.out.println("Calculated UTILITY:");
            System.out.println(weight1*exploration);
            System.out.println(weight2*distance);
            System.out.println(weight3*distribution);
            System.out.println(weight4*lineOfSight);
            System.out.println(weight5*closeToWall);
            System.out.println(weight6*turnDistance);
        }
        return weight1*exploration - weight2*distance-weight3*distribution+weight4*lineOfSight-weight5*closeToWall - tooNear - weight6*turnDistance;
    }
}
