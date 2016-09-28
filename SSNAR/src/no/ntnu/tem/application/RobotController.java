/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package no.ntnu.tem.application;

import no.ntnu.tem.robot.Robot;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import no.ntnu.tem.communication.Language;

/**
 * This class holds a List over all the robots in the system. It holds methods
 * for adding and removing robots as well as searching for special robots.
 *
 * @author Thor Eivind and Mats (Master 2016 @ NTNU)
 */
public class RobotController {

    private final ObservableList<Robot> robotList;
    private final ObservableList<String[]> availableRobots;
    private final boolean debug = false;

    /**
     * Constructor of the class RobotController, starts off with an empty list
     * of robots.
     */
    public RobotController() {
        this.robotList = FXCollections.observableArrayList();
        this.availableRobots = FXCollections.observableArrayList();
    }

    /**
     * Method that returns the robotlist containing the connected robots
     *
     * @return the list
     */
    public ObservableList<Robot> getRobotList() {
        return robotList;
    }

    /**
     * Method that adds a discovered robot to the available robot list
     *
     * @param id the robots id
     * @param name the robots name
     */
    public void addAvailableRobot(int id, String name) {
        String[] robot = new String[]{"" + id, name};
        availableRobots.add(robot);
    }

    /**
     * Method that adds a discovered robot to the available robot list
     *
     * @param robot a String array where robot[0] is the id and robot[1] is the
     * name of the robot
     */
    public void addAvailableRobot(String[] robot) {
        availableRobots.add(robot);
    }

    /**
     * Method that returns the available robot list
     *
     * @return the list
     */
    public ObservableList<String[]> getAvailableRobotList() {
        return availableRobots;
    }

    /**
     * Method that adds a robot to the system
     *
     * @param robotID The robots id
     * @param name The robots name
     * @param width The robots physical width (cm)
     * @param length The robots physical length (cm)
     * @param messageDeadline The robots message deadline (min update rate[ms])
     * @param axleOffset Axle offset lengthwise
     * @param towerOffset Tower offset [a,b] a-lengthwise, b-crosswise
     * @param sensorOffset Sensor offset [a,b,c,d,..] from centre (radius)
     * @param irHeading IR heading relative to 0 deg (straight forward)
     * @return true if successful
     */
    public boolean addRobot(int robotID, String name, int width, int length, int messageDeadline,
            int axleOffset, int[] towerOffset, int[] sensorOffset, int[] irHeading) {
        if (getRobot(robotID) == null) {
            try {
                Robot robot = new Robot(robotID, name, width, length, messageDeadline,
                        axleOffset, towerOffset, sensorOffset, irHeading);
                robotList.add(robot);
                if (debug) {
                    System.out.println("Robot <" + robot.getName() + "> created!");
                }
                return true;
            } catch (Exception e) {
                e.printStackTrace();
                return false;
            }
        }
        return false;
    }

    /**
     * Method that adds a measurement to a robot
     *
     * @param robotName name of the robot
     * @param measuredOrientation measured orientation
     * @param measuredPosition measured position
     * @param irHeading angle of the robot tower
     * @param irData data from IR sensors
     * @return returns true if everything is ok.
     */
    public boolean addMeasurment(String robotName, int measuredOrientation, int[] measuredPosition, int irHeading, int[] irData) {
        Robot robot = getRobot(robotName);
        if (robot == null) {
            return false;
        }
        if (debug) {
            System.out.println("Robot <" + robot.getName() + "> updated!");
        }
        return robot.addMeasurement(measuredOrientation, measuredPosition, irHeading, irData);
    }

    /**
     * Method that updates a robots status
     *
     * @param robotName the robot name
     * @param status the new status
     * @return
     */
    public boolean addStatus(String robotName, String status) {
        Robot robot = getRobot(robotName);
        if (robot == null) {
            return false;
        }
        if (debug) {
            System.out.println("Robot <" + robot.getName() + "> status updated!");
        }
        switch (status) {
            case Language.STATUS_IDLE:
                if (debug) {
                    System.out.println("idl det da");
                }
                robot.setBusy(false);
                return true;
            case Language.STATUS_HIT:
                return true;
            case Language.STATUS_BUSY:
                if (debug) {
                    System.out.println("busy det da");
                }
                robot.setBusy(true);
                return true;
            default:
                return false;
        }
    }

    /**
     * Method that returns the robot, if existing, with the given name
     *
     * @param name the robot name
     * @return the first robot with the given name
     */
    public Robot getRobot(String name) {
        for (Robot r : robotList) {
            if (r.getName().equalsIgnoreCase(name)) {
                return r;
            }
        }
        return null;
    }

    /**
     * Method that returns the robot, if existing, with the given robot id
     *
     * @param robotID the robot id
     * @return the first robot with the given id
     */
    public Robot getRobot(int robotID) {
        for (Robot r : robotList) {
            if (r.getId() == robotID) {
                return r;
            }
        }
        return null;
    }

    /**
     * Method that removes the robot, if existing, with the given name
     *
     * @param robotID the robot id.
     * @return true if successful
     */
    public boolean removeRobot(int robotID) {
        for (Robot r : robotList) {
            if (r.getId() == robotID) {
                robotList.remove(r);
                return true;
            }
        }
        return false;
    }
}
