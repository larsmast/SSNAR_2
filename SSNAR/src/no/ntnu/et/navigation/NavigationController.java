/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.navigation;

/**
 * This class checks for commands in all of the Navigation Robots and sends
 * them to the robots via the Application. If a robot has no new commands this
 * class creates a new worker thread in RobotTaskManager to find a new task for
 * the robot.
 * 
 * @author Eirik Thon
 */

import no.ntnu.et.navigation.RobotTaskManager;
import java.util.ArrayList;
import java.util.HashMap;
import no.ntnu.et.general.Position;
import no.ntnu.et.map.GridMap;
import no.ntnu.et.map.MapLocation;
import no.ntnu.tem.application.Application;
import no.ntnu.tem.application.RobotController;
import no.ntnu.tem.robot.Robot;

/**
 *
 * @author Eirik Thon
 */

public class NavigationController extends Thread{
    

    private CollisionManager collisionManager;

    private RobotTaskManager robotTaskManager;

    private RobotController robotController;

    private Application application;

    private HashMap<String, NavigationRobot> navigationRobots;

    private ArrayList<String> robotNames;
    
    private boolean paused;
    
    private boolean debug = false;

    public NavigationController(RobotController robotController, Application application, GridMap map) {
        this.robotController = robotController;
        this.application = application;
        robotTaskManager = new RobotTaskManager(map);
        collisionManager = new CollisionManager(map, robotController);
        collisionManager.setName("Collision management");
        robotNames = new ArrayList<String>();
        navigationRobots = new HashMap<String, NavigationRobot>();
    }
    
    
    public void addRobot(String robotName, int id) {
        Position currentPosition = new Position(robotController.getRobot(robotName).getPosition());
        NavigationRobot newNavRobot = new NavigationRobot(currentPosition);
        navigationRobots.put(robotName, newNavRobot);
        robotNames.add(robotName);
        collisionManager.addRobot(robotName, newNavRobot);
    }
    
    public void removeRobot(String robotName) {
        robotNames.remove(robotName);
        collisionManager.removeRobot(robotName);
    }
    
    @Override
    public void start(){
        if(!isAlive()){
            super.start();
            collisionManager.start();
        }
        else{
            collisionManager.unpause();
        }
        resumeAllRobots();
        paused = false;
    }

    public void pause(){
        collisionManager.pause();
        stopAllRobots();
        paused = true;
    }
    
    public void quit(){
        paused = true;
    }
    
    private void stopAllRobots(){
        for(int i = 0; i < robotNames.size(); i++){
            String name = robotNames.get(i);
            Robot robot = robotController.getRobot(name);
            int id = robot.getId();
            application.pauseRobot(id);
        }
    }
    
    private void resumeAllRobots(){
        for(int i = 0; i < robotNames.size(); i++){
            String name = robotNames.get(i);
            Robot robot = robotController.getRobot(name);
            int id = robot.getId();
            application.unPauseRobot(id);
        }
    }
    
    @Override
    public void run() {
        setName("Navigation Controller");
        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        boolean finished = false;
        while(!finished) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                stopAllRobots();
                break;
            }
            if(paused){
                continue;
            }
            
            for(int i = 0; i < robotNames.size(); i++){
                String name = robotNames.get(i);
                // This disables control commands to the SlamRobot
                if (name.equals("SlamRobot")) {
                    continue;
                }
                Robot applicationRobot = robotController.getRobot(name);
                int id = applicationRobot.getId();
                if(navigationRobots.get(name).hasNewPriorityCommand()){
                    int[] nextCommand = navigationRobots.get(name).getPriorityCommand();
                    if(debug){
                        System.out.println(name + ": Executing next command, ROTATION " + nextCommand[0] + ", DISTANCE "+ nextCommand[1]);
                    }
                    application.writeCommandToRobot(id, name, nextCommand[0], nextCommand[1]); 
                }
                
                else if(!applicationRobot.isBusy() && !navigationRobots.get(name).isInCollisionManagement()){
                    if(navigationRobots.get(name).hasMoreWaypoints()){
                        // Get next target
                        Position nextWaypoint = navigationRobots.get(name).getNextWaypoint();
                        // Get current obot location and orientation
                        Position currentPosition = new Position(applicationRobot.getPosition());
                        int currentOrientation = applicationRobot.getRobotOrientation();
                        // Command the robot to move to the next waypoint along its path
                        int[] newCommand = findCommandToTargetPoint(nextWaypoint, currentPosition, currentOrientation);
                        if(debug){
                            System.out.println(name+ ": Executing next command, ROTATION " + newCommand[0] + ", DISTANCE "+ newCommand[1]);
                        }
                        application.writeCommandToRobot(id, name, newCommand[0], newCommand[1]); 
                    }
                    else if(!robotTaskManager.isWorkingOnTask(name)){
                        if(debug){
                            System.out.println(name + ": Idle. Searching for best target");
                        }
                        robotTaskManager.createNewTask(robotController.getRobot(name), navigationRobots.get(name), name);
                    }
                }
            }
        }
    }
    
    static int[] findCommandToTargetPoint(Position target, Position currentPosition, int robotHeading){
        int distance = (int)Position.distanceBetween(currentPosition, target);
        int rotation = ((int)Position.angleBetween(currentPosition, target).getValue()-robotHeading+360)%360;
        if(rotation > 180){
            rotation -= 360;
        }
        int[] command = {rotation, distance};
        return command;
    }
}