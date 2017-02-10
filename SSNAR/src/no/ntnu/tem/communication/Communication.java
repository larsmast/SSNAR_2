/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Thor Eivind Andersen and Mats Rødseth (Master 2016 @ NTNU)
 */
package no.ntnu.tem.communication;

import gnu.io.NoSuchPortException;
import gnu.io.PortInUseException;
import gnu.io.UnsupportedCommOperationException;
import java.io.IOException;
import java.util.LinkedList;
import no.ntnu.tem.application.RobotController;

/**
 * This class represents the initializer in the communication package. It holds
 * the different instantiated objects, and it provides methods for sending
 * communication commands.
 *
 * @author Thor Eivind and Mats (Master 2016 @ NTNU)
 */
public class Communication {

    private final Inbox inbox;
    private final InboxReader inR;
    private final SerialCommunication serialcom;
    private final ListPorts lp;
    private String comPort = null;

    /**
     * Constructor of the class Communication
     *
     * @param rc the RobotController object
     */
    public Communication(RobotController rc) {
        this.inbox = new Inbox();
        this.inR = new InboxReader(inbox, rc);
        this.serialcom = new SerialCommunication(inbox);
        this.lp = new ListPorts();
    }

    /**
     * Method for starting the communication module
     *
     * @param port The port to connect to, e.g. COM1, COM2, COM3 on Windows
     * @throws gnu.io.UnsupportedCommOperationException .
     * @throws gnu.io.PortInUseException if the port is busy
     * @throws java.io.IOException .
     * @throws gnu.io.NoSuchPortException if the port doesn't exist
     */
    public void startCommunication(String port)
            throws UnsupportedCommOperationException, PortInUseException,
            IOException, NoSuchPortException {
        serialcom.connect(port);
        if (serialcom.isAlive()) {
        } else {
            serialcom.start();
        }
    }

    /**
     * Method that returns a list containing available com ports
     *
     * @return the list
     */
    public LinkedList<String> listPorts() {
        return lp.listPorts();
    }

    /**
     * Method that initiates the inbox reader
     */
    public void startInboxReader() {
        if (!inR.isAlive()) {
            inR.start();
        }
    }

    /**
     * Method that returns the name of the current com in use
     *
     * @return the name of the current com-port
     */
    public String getComPort() {
        return this.comPort;
    }

    /**
     * Method that sets the name of the current com port to use
     *
     * @param comPort name of the comport
     */
    public void setComPort(String comPort) {
        this.comPort = comPort;
    }

    /**
     * Method that returns the inbox object
     *
     * @return the inbox
     */
    public Inbox getInbox() {
        return this.inbox;
    }

    /**
     * Method that returns the serial communication object
     *
     * @return the serial communication
     */
    public SerialCommunication getSerialCommunication() {
        return serialcom;
    }

    /**
     * Method that wraps a message and sends it to a robot
     *
     * @param robotID the receiving robot
     * @param orientation the robots wanted orientation
     * @param distance the distance to move
     */
    public void sendOrderToRobot(int robotID, double orientation, double distance) {
        switchToCommandMode();
        serialcom.send(MessageHandler.wrapSwitchToRobot(robotID)); //*switch robotID
        serialcom.send(MessageHandler.wrapRobotOrder(orientation, distance));
    }
    
    public void sendStartDebugToRobot(int robotID) {
        switchToCommandMode();
        serialcom.send(MessageHandler.wrapSwitchToRobot(robotID)); //*switch robotID
        serialcom.send("{" + "D" + "," + "STA" + "}");
    }
    
    public void sendStopDebugToRobot(int robotID) {
        switchToCommandMode();
        serialcom.send(MessageHandler.wrapSwitchToRobot(robotID)); //*switch robotID
        serialcom.send("{" + "D" + "," + "STO" + "}");
    }
    
    /**
     * Method that confirms that the server has received the handshake
     *
     * @param robotID the robots ID
     */
    public void confirmHandshake(int robotID) {
        switchToCommandMode();
        serialcom.send(MessageHandler.wrapSwitchToRobot(robotID)); //*switch robotID
        serialcom.send(MessageHandler.wrapStatus("CON"));
    }

    /**
     * Method that pauses the robot
     *
     * @param robotID the robots ID
     */
    public void pauseRobot(int robotID) {
        switchToCommandMode();
        serialcom.send(MessageHandler.wrapSwitchToRobot(robotID));
        serialcom.send(MessageHandler.wrapStatus("PAU"));
    }

    /**
     * Method that unpauses the robot
     *
     * @param robotID the robots ID
     */
    public void unPauseRobot(int robotID) {
        switchToCommandMode();
        serialcom.send(MessageHandler.wrapSwitchToRobot(robotID));
        serialcom.send(MessageHandler.wrapStatus("UNP"));
    }

    /**
     * Method that confirms that the robot is finished
     *
     * @param robotID the robots ID
     */
    public void confirmRobotFinished(int robotID) {
        switchToCommandMode();
        serialcom.send(MessageHandler.wrapSwitchToRobot(robotID));
        serialcom.send(MessageHandler.wrapStatus("FIN"));
    }

    /**
     * Method that starts scanning for new robots
     */
    public void startScanForNewRobots() {
        switchToCommandMode();
        serialcom.send("scan");
    }

    /**
     * Method that stops scanning for new robots
     */
    public void stopScanForNewRobots() {
        switchToCommandMode();
        serialcom.send("stop");
    }

    /**
     * Method that initiates a connection to a specified robot
     *
     * @param robotID the robots id
     */
    public void connectToRobot(int robotID) {
        switchToCommandMode();
        serialcom.send(MessageHandler.wrapConnectToRobot(robotID));
    }

    /**
     * Method that cancels the connection to the specified robot
     *
     * @param robotID the robots id
     */
    public void disconnectRobot(int robotID) {
        switchToCommandMode();
        serialcom.send(MessageHandler.wrapDisconnectRobot(robotID));
    }

    /**
     * Method that switches selected robot from the current one to the specified
     * one
     *
     * @param robotID the robots id
     */
    public void switchToRobot(int robotID) {
        switchToCommandMode();
        serialcom.send(MessageHandler.wrapSwitchToRobot(robotID));
    }

    /**
     * Method that sets the NRF to command mode
     */
    public void switchToCommandMode() {
        serialcom.send("*");
    }

    /**
     * Method that tells the NRF to list found robots
     */
    public void listAvailableRobots() {
        switchToCommandMode();
        serialcom.send("list");

    }

    /**
     * Method that resets the NRF dongle
     */
    public void resetDongle() {
        switchToCommandMode();;
        serialcom.send("reset");
    }
}
