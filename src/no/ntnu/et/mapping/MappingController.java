/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.mapping;

import java.util.ArrayList;
import org.ejml.simple.SimpleMatrix;
import java.util.HashMap;
import java.util.List;
import no.ntnu.tem.application.RobotController;
import no.ntnu.et.map.GridMap;
import no.ntnu.et.map.MapLocation;
import no.ntnu.et.general.Angle;
import no.ntnu.et.general.Pose;
import no.ntnu.et.general.Position;
import no.ntnu.et.navigation.NavigationRobot;
import no.ntnu.tem.robot.Robot;
import no.ntnu.et.mapping.TransformationAlg;

/**
 * This class creates the map from the measurements. See (Thon 2016) for more
 * information on how it works.
 *
 * @author Eirik Thon
 */
public class MappingController extends Thread {

    ArrayList<String> robotNames;
    private GridMap map;

    //test LMS
    private GridMap dockMap;
    //

    private RobotController robotController;
    private HashMap<String, MeasurementHandler> measurementHandlers;
    private Object nameLock = new Object();
    private boolean paused;
    private Thread mapCleaner;
    private NavigationRobot navRobot;
    private final boolean debug = false;

    /**
     * Constructor
     *
     * @param rc
     * @param map
     */
    // Edited by LMS for docking purposes
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
     *
     * @param name
     */
    public void addRobot(String name) {
        robotNames.add(name);
        int[] initialRobotPose = robotController.getRobot(name).getInitialPosition();
        Pose initialPose = new Pose(initialRobotPose[0], initialRobotPose[1], initialRobotPose[2]);
        MeasurementHandler newHandler = new MeasurementHandler(robotController.getRobot(name), initialPose);
        measurementHandlers.put(name, newHandler);
        int[] initialPosition = {(int) Math.round(initialPose.getPosition().getXValue()), (int) Math.round(initialPose.getPosition().getYValue())};
        robotController.getRobot(name).setPosition(initialPosition);
        robotController.getRobot(name).setRobotOrientation((int) Math.round(initialPose.getHeading().getValue()));
        robotController.getRobot(name).setDestination(initialPosition);
        map.resize(initialPose.getPosition());
    }

    /**
     * Removes a robot from the mapping process
     *
     * @param name
     */
    public void removeRobot(String name) {
        robotNames.remove(name);
        measurementHandlers.remove(name);
    }

    /**
     * Starts the mapping.
     */
    @Override
    public void start() {
        if (!isAlive()) {
            super.start();
        }
        paused = false;
    }

    /**
     * Pauses the mapping
     */
    public void pause() {
        paused = true;
    }

    /**
     * Returns if the mapping is running or paused
     *
     * @return
     */
    public boolean isRunning() {
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
        int rangeScan = 0;
        int numberOfScans = 530;
        boolean scanButton = false;
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
            for (int i = 0; i < robotNames.size(); i++) {
                String name = robotNames.get(i);
                Robot robot = robotController.getRobot(name);
                if (measurementHandlers.get(name).updateMeasurement() == false) {
                    continue;
                }

                Position robotPosition = measurementHandlers.get(name).getRobotPosition();
                Angle robotAngle = measurementHandlers.get(name).getRobotHeading();

                int[] position = {(int) Math.round(robotPosition.getXValue()), (int) Math.round(robotPosition.getYValue())};
                robot.setPosition(position);
                robot.setRobotOrientation((int) Math.round(robotAngle.getValue()));

                // Find the location of the robot in the map
                map.resize(robotPosition);
                MapLocation robotLocation = map.findLocationInMap(robotPosition);

                Sensor[] sensors = measurementHandlers.get(name).getIRSensorData();
                int sensorOneValue = 0;

                int sensCount = 0;

                for (Sensor sensor : sensors) {
                    sensCount++;

                    boolean tooClose = false;

                    // Check the distance between the position of the measurement and all the other robots
                    for (int j = 0; j < robotNames.size(); j++) {
                        String otherName = robotNames.get(j);
                        int[] otherPositionInt = robotController.getRobot(otherName).getPosition();
                        Position otherPosition = new Position(otherPositionInt[0], otherPositionInt[1]);
                        if (Position.distanceBetween(otherPosition, sensor.getPosition()) < 10) {
                            tooClose = true;
                            break;
                        }
                    }

                    // The measurement is only added to the map if it is at a certain distance to the other robots
                    if (!tooClose) {
                        int[] irheading = measurementHandlers.get(name).getSensorAngel();
                        int[] irdata = measurementHandlers.get(name).getCurrentMeasurement().getIRdata();

                        map.resize(sensor.getPosition());
                        MapLocation measurementLocation = map.findLocationInMap(sensor.getPosition());
                        Position target = new Position(measurementLocation.getColumn(), measurementLocation.getRow());
                        if (sensor.isMeasurement()) {
                            map.addMeasurement(measurementLocation, true);
                        }
                        // RangeScan near base for MapMatching on return sends once when the tower turns
                        if (robot.isRangeScanBase() && !robot.isGoingHome()) {
                            rangeScan++;
                            robot.addToREF(target);
                            if (rangeScan == numberOfScans) {
                                endScan(robot);
                                rangeScan = 0;
                            }
                        }
                        // RangeScan near base for matching with ref.scan
                        //System.out.println("Sensor: " + sensCount + " sends: " + measurementLocation.getColumn() + " " + measurementLocation.getRow());
                        if (robot.isGoingHome() && robot.isRangeScanBase()) {
                            rangeScan++;
                            System.out.println(rangeScan);
                            robot.addToNEW(target);
                            System.out.println("Sensor: " + sensCount + " sends: " + irdata[sensCount-1]);
                            if (rangeScan == numberOfScans) {
                                endScan(robot);
                                rangeScan = 0;
                                initDocking(robot);
                            }
                        }
                        // Find distance to backwall
                        if (robot.getAdjustRobot() < -2 && sensCount == 2 && irheading[0] < 2) {
                            findWall(robot, 2, irdata);
                        }
                        
                        if (robot.getAdjustRobot() < -2 && sensCount == 2 && irheading[0] > 86) {
                            findWall(robot, 3, irdata);
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

    private void findWall(Robot robot, int irSensor, int[] irdata) {
        if (irdata[irSensor] != 0) {
            robot.setBackUpDist((int) (irdata[irSensor] /** 0.80*/));
            System.out.println("Finding distance to back wall! : " + irdata[irSensor] +" from sensor " + irSensor);
            robot.setRobotAligned(true);
            robot.setAdjustRobot(4);
        } else {
            robot.setAdjustRobot(7);
        }
    }
    
    /**
     * Returns all map locations in a straight line between two MapLocations.
     * Uses Bresenham's line algorithm
     *
     * @param loc1
     * @param loc2
     * @return
     */
    public static ArrayList<MapLocation> getLineBetweenPoints(MapLocation loc1, MapLocation loc2) {
        int dx = loc2.getColumn() - loc1.getColumn();
        int dy = loc2.getRow() - loc1.getRow();
        double angle = Math.toDegrees(Math.atan2(dy, dx));
        if (angle < 0) {
            angle += 360;
        }
        int oct = MapLocation.getOctant(angle);
        MapLocation locOct = MapLocation.switchToOctantZeroFrom(oct, new MapLocation(dy, dx));
        ArrayList<MapLocation> lineOct = MappingController.bresenham(new MapLocation(0, 0), locOct);
        ArrayList<MapLocation> line = new ArrayList<MapLocation>();
        for (MapLocation loc : lineOct) {
            line.add(MapLocation.sum(loc1, MapLocation.switchFromOctantZeroTo(oct, loc)));
        }
        return line;
    }

    /*
    * Ending scan
     */
    private void endScan(Robot robot) {
        robot.setHome(false);
        robot.setDock(false);
        robot.setRangeScanBase(false);
    }

    /*
    * initiate docking
     */
    private void initDocking(Robot robot) {
        robot.setRealPose(ScanMatching(robot.S_NEW, robot.S_REF, robot.getPosition()));
        robot.setAdjustDirection((int) ((int) ScanMatchingDeg(robot.S_NEW, robot.S_REF, robot.getPosition()))*5);
        robot.setAdjustRobot(1);
        robot.resetNew();
    }

    /**
     * Bresenham's line algorithm. Found on
     * https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
     *
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

    /*
    * Returning rotation
     */
    public double ScanMatchingDeg(List<Position> S_new, List<Position> S_ref, int[] currentPose) {
        List<Position[]> CLOSESTPOINT = new ArrayList<Position[]>();
        ArrayList<Position> Pvalues = new ArrayList<>();
        ArrayList<Position> Qvalues = new ArrayList<>();
        ArrayList<Position> clonedS_ref = new ArrayList<Position>();
        clonedS_ref.addAll(S_ref);
        int n = 0;
        if (debug) {
            System.out.println("Pairing sets of points. Num of new points = " + S_new.size() + " Num of ref points: " + S_ref.size());
        }
        for (Position poseNew : S_new) {
            Position solution = FindClosestPoint(poseNew, clonedS_ref);
            CLOSESTPOINT.add(new Position[]{poseNew, solution});
            if (solution != null) {
                Pvalues.add(poseNew);
                Qvalues.add(solution);
                n++;
                clonedS_ref.remove(solution);
                //System.out.println(clonedS_ref.size());
            }
        }
        SimpleMatrix P = generateMatrixA(Pvalues, n);
        SimpleMatrix Q = generateMatrixA(Qvalues, n);
        //TransformationAlg ka = new TransformationAlg(P, Q);
        TransformationAlg ka = new TransformationAlg(Q, P);
        SimpleMatrix rot = ka.getRotation();
        SimpleMatrix transVect = ka.getTranslationVek();
        SimpleMatrix p = generatePositionVector(currentPose);
        SimpleMatrix T_h = homogeneousTransformation(rot, transVect);
        SimpleMatrix realPose = T_h.mult(p);
        if (debug) {
            P.print();
            Q.print();
            p.print();
            rot.print();
            transVect.print();
            realPose.print();
            T_h.print();
        }
        double adjustment = Math.toDegrees(Math.asin(rot.get(0, 1)));
        System.out.println("Correction angle: " + adjustment );
        return adjustment;
    }

    /*
    *  Matching two scans
     */
    public SimpleMatrix ScanMatching(List<Position> S_new, List<Position> S_ref, int[] currentPose) {
        List<Position[]> CLOSESTPOINT = new ArrayList<Position[]>();
        ArrayList<Position> Pvalues = new ArrayList<>();
        ArrayList<Position> Qvalues = new ArrayList<>();
        ArrayList<Position> clonedS_ref = new ArrayList<Position>();
        clonedS_ref.addAll(S_ref);
        int n = 0;
        if (debug) {
            System.out.println("Pairing sets of points. Num of new points = " + S_new.size() + " Num of ref points: " + S_ref.size());
        }
        for (Position poseNew : S_new) {
            Position solution = FindClosestPoint(poseNew, clonedS_ref);
            CLOSESTPOINT.add(new Position[]{poseNew, solution});
            if (solution != null) {
                Pvalues.add(poseNew);
                Qvalues.add(solution);
                n++;
                clonedS_ref.remove(solution);
                //System.out.println(clonedS_ref.size());
            }
        }
        SimpleMatrix P = generateMatrixA(Pvalues, n);
        SimpleMatrix Q = generateMatrixA(Qvalues, n);
        //TransformationAlg ka = new TransformationAlg(P, Q);
        TransformationAlg ka = new TransformationAlg(Q, P);
        SimpleMatrix rot = ka.getRotation();
        SimpleMatrix transVect = ka.getTranslationVek();
        SimpleMatrix p = generatePositionVector(currentPose);
        SimpleMatrix T_h = homogeneousTransformation(rot, transVect);
        SimpleMatrix realPose = T_h.mult(p);
        if (debug) {
            P.print();
            Q.print();
            p.print();
            rot.print();
            transVect.print();
            realPose.print();
            T_h.print();
        }
        T_h.print();
        realPose.print();
        double adjustment = Math.toDegrees(Math.acos(rot.get(0, 0)));
        System.out.println("Error in orientation: " + adjustment);
        System.out.println("NUMBER OF MATCHES : " + n);
        return realPose;
    }

    public SimpleMatrix homogeneousTransformation(SimpleMatrix r, SimpleMatrix t) {
        double[][] m = new double[3][3];
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                m[i][j] = r.get(i, j);

            }
        }
        m[0][2] = t.get(0, 0);
        m[1][2] = t.get(1, 0);
        m[2][2] = 1;

        return new SimpleMatrix(m);
    }

    /*
    * Generate matrix in correct for for sending to kabshAgl
     */
    public SimpleMatrix generateMatrixA(ArrayList<Position> pose, int row) {
        double[][] data = new double[row][2];
        for (int i = 0; i < row; i++) {
            data[i][0] = pose.get(i).getXValue();
            data[i][1] = pose.get(i).getYValue();
        }

        return new SimpleMatrix(data);
    }

    public SimpleMatrix generatePositionVector(int[] pose) {
        double[][] data = new double[3][1];
        data[0][0] = pose[0];
        data[1][0] = pose[1];
        data[2][0] = 1;

        return new SimpleMatrix(data);
    }

    public SimpleMatrix makePositionVector(Position pose) {
        double[][] data = new double[2][1];
        data[0][0] = pose.getXValue();
        data[1][0] = pose.getYValue();
        data[1][0] = 1;
        return new SimpleMatrix(data);
    }

    /*
    * Searches for a position in list (NOT CURRENTLY USED)
     */
    public boolean containsPosition(List<Position> list, Position pose) {
        if (list.size() > 0) {
            for (int i = 0; i < list.size(); i++) {
                if (list.get(i).getXValue() == pose.getXValue() && list.get(i).getYValue() == pose.getYValue()) {
                    //System.out.println("Index: " + i + " is removed.");
                    return true;
                }
                return false;
            }
        }
        return false;
    }

    /*
    * Function that matches points in set. Takes in a one point and a set, this is done one point at a time so it is easy to manipulate data futher on.
     */
    private Position FindClosestPoint(Position poseNew, ArrayList<Position> S_ref) {
        Position closestPoint = new Position(0, 0);
        double bestDistance = 1000;
        double thresholdDist = 7;
        for (Position P_ref : S_ref) {
            double newDistance = Position.distanceBetween(poseNew, P_ref);
            if (newDistance < bestDistance && newDistance < thresholdDist) {
                bestDistance = newDistance;
                closestPoint = P_ref;
            }
        }
        if (bestDistance < 1000) {
            return closestPoint;

        }
        return null;

    }

    /*
    Making matrix
     */
    public SimpleMatrix setMatrix(ArrayList<Position> list) {
        SimpleMatrix m1 = new SimpleMatrix(new double[list.size()][1]);
        for (int i = 0; i < list.size(); i++) {
            m1.set(0, i, (double) list.get(i).getXValue());
            //m1.set(1, i, (double) list.get(i).getYValue());
        }
        return m1;
    }

    /**
     * Worker thread used for filling in unexplored gaps in the map.
     *
     */
    private class MapCleaningWorker implements Runnable {

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
                if (cleanUpCountDown == 100) {
                    cleanUpCountDown = 0;
                    map.cleanUp();
                }
            }
        }
    }
}
