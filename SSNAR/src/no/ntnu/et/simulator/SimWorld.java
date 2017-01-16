/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.simulator;

import no.ntnu.et.general.Utilities;
import no.ntnu.et.general.Pose;
import no.ntnu.et.general.Position;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;
import no.ntnu.et.general.Line;

/**
 * This class represents the environment of the simulator. It contains all the
 * features and all the robots. The scale of the world is in cm, meaning that
 * if the width is 600, it represents an environment that is 6 meters across
 * 
 * @author Eirik Thon
 */
public class SimWorld {
    private int width;
    private int height;
    ArrayList<Feature> features;
    ArrayList<SimRobot> robots;
    ArrayList<Integer> robotIDs;
    private ArrayList<String[]> possibleRobotNames;

    /**
     * Default constructor
     */
    public SimWorld() {
        width = 0;
        height = 0;
        features = new ArrayList<Feature>(0);
        robots = new ArrayList<SimRobot>(0);
        robotIDs = new ArrayList<Integer> (0);
        possibleRobotNames = new ArrayList<String[]>();
        populateRobotNames();
    }
    
    /**
     * Returns the height of the map
     * @return height
     */
    int getMapHeight() {
        return height;
    }
    
    /**
     * Returns the width of the map
     * @return width
     */
    int getMapWidth() {
        return width;
    }
    
    
    /**
    * Creates and returns a copy of the features in the SimWorld.
    * This function is safe to call concurrently.
    * @return ArrayList<String> robotNamesCopy
    */
    ArrayList<Feature> getFeatures() {
        return features;   
    }
    
    ArrayList<SimRobot> getRobots() {
        return robots;
    }
    private void populateRobotNames() {
        possibleRobotNames.add(new String[]{"0", "SlamRobot"});
        possibleRobotNames.add(new String[]{"1", "Arduino1"});
        possibleRobotNames.add(new String[]{"2", "NXT1"});
        possibleRobotNames.add(new String[]{"3", "AVR2"});
        possibleRobotNames.add(new String[]{"4", "Arduino2"});
        possibleRobotNames.add(new String[]{"5", "NXT2"});
        possibleRobotNames.add(new String[]{"6", "AVR3"});
        possibleRobotNames.add(new String[]{"7", "Arduino3"});
        possibleRobotNames.add(new String[]{"8", "NXT3"});
        possibleRobotNames.add(new String[]{"9", "AVR4"});
    }
    
    void createRobot(Pose initialPose) {
        String[] identification = possibleRobotNames.remove(0);
        int id = Integer.parseInt(identification[0]);
        String name = identification[1];
        SimRobot newRobot;
        if (name.equals("SlamRobot")) {
            newRobot = new SlamRobot(this, initialPose, name, id);
            System.out.println("SlamRobot created");
        } else {
            newRobot = new BasicRobot(this, initialPose, name, id);
            System.out.println("SimRobot created");
        }
        robotIDs.add(id);
        robots.add(newRobot);
    }
    
    
    boolean checkIfPositionIsFree(Position center, int ignoredRobotId) {
        double collisionSize = 5; // 5 cm
        for(int i = 0; i < features.size(); i++){
            // Create vector from feature start to position
            Feature feature = features.get(i);
            Position projection = Utilities.getOrthogonalProjection(center, Line.convertFeatureToLine(feature));
            if(projection == null){
                continue;
            }
            // Check for collision
            if (Position.distanceBetween(center, projection) < collisionSize) {
                return false;
            }
        }
        for (int i = 0; i < robotIDs.size(); i++){
            if(robotIDs.get(i) == ignoredRobotId){
                continue;
            }
            Position robotPosition = robots.get(robotIDs.get(i)).getPose().getPosition();
            if(Position.distanceBetween(center, robotPosition) < collisionSize*2){
                return false;
            }
        }
        return true;
    }
    
    double findNearestIntersection(Line line, int ignoredRobotId){
        double shortestMeasurement = Double.POSITIVE_INFINITY;
        for(int i = 0; i < features.size(); i++){
            Line featureLine = Line.convertFeatureToLine(features.get(i));
            double newIRMeasurement = Utilities.lineLineIntersection(line, featureLine);
            // Add only the measurement of the nearest feature
            if (newIRMeasurement != 0 && newIRMeasurement < shortestMeasurement) {
                shortestMeasurement = newIRMeasurement;
            }
        }
        for (int i = 0; i < robotIDs.size(); i++){
            if(robotIDs.get(i) == ignoredRobotId){
                continue;
            }
            double newIRMeasurement = Utilities.lineCircleIntersection(line, robots.get(i).getPose().getPosition(), 5);
            // Add only the measurement of the nearest feature
            if (newIRMeasurement != 0 && newIRMeasurement < shortestMeasurement) {
                shortestMeasurement = newIRMeasurement;
            }
        }
        if(shortestMeasurement == Double.POSITIVE_INFINITY){
            shortestMeasurement = 0;
        }
        return shortestMeasurement;
    }
    
    SimRobot getRobot(int id) {
        return robots.get(id);
    }
    
    /**
     * Creates and returns a copy of the robot names stored in the SimWorld.
     * This function is safe to call concurrently.
     * @return ArrayList<String> robotNamesCopy
     */
    ArrayList<Integer> getRobotIDs() {
        return robotIDs;
    }
    
    /**
     * Sets the map size and adds all features in the map based upon data found
     * in the text file given by the input parameter filename
     * @param filename The string filename specifies the map file
     */
    void initMap(String filename){
        features = new ArrayList<Feature>();
        try{
            File map = new File(filename);
            Scanner sc = new Scanner(map);
            
            initWorldSize(sc.next());
            while(sc.hasNext()){
                String featurePositionString = sc.next();
                String[] stringParts = featurePositionString.split(":");
                Position start = Utilities.string2Position(stringParts[0]);
                Position end = Utilities.string2Position(stringParts[1]);
                Feature newFeature = new Feature(start, end);
                features.add(newFeature);
            }
            sc.close();
            addBorderFeatures();
        }
        catch(FileNotFoundException e){
            System.out.println("File not found");
        }        
    }
    
    /**
     * Reads the contents of the input parameter size and updates the width
     * and height of the world based upon this
     * @param size Sting on the format "width:height"
     */
    private void initWorldSize(String size){
        String[] stringParts = size.split(":");
        width = Integer.parseInt(stringParts[0]);
        height = Integer.parseInt(stringParts[1]);
    }
    
    /**
     * Creates features that lies along the edges of the map and adds them to
     * the other features
     */
    private void addBorderFeatures(){
        Feature leftBorder = new Feature(0,0,0,height);
        features.add(leftBorder);
        Feature rightBorder = new Feature(width,0,width,height);
        features.add(rightBorder);
        Feature topBorder = new Feature(0,0,width,0);
        features.add(topBorder);
        Feature bottomBorder = new Feature(0,height,width,height);
        features.add(bottomBorder);
    }
}
