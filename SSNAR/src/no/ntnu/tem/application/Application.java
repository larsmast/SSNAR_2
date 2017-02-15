/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Thor Eivind Andersen and Mats Rødseth (Master 2016 @ NTNU)
 */
package no.ntnu.tem.application;

import no.ntnu.tem.gui.MapGraphic;
import gnu.io.NoSuchPortException;
import gnu.io.PortInUseException;
import gnu.io.UnsupportedCommOperationException;
import java.awt.Desktop;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.LinkedList;
import java.util.logging.Level;
import java.util.logging.Logger;
import javafx.collections.ListChangeListener;
import javafx.collections.ObservableList;
import no.ntnu.et.map.GridMap;
import no.ntnu.et.navigation.NavigationController;
import no.ntnu.tem.communication.Communication;
import no.ntnu.tem.gui.MainGUI;
import no.ntnu.et.simulator.Simulator;
import no.ntnu.et.mapping.MappingController;
import no.ntnu.tem.robot.Robot;

/**
 * This class is the main class of the program. It connects the packages in the
 * project, and instantiates the different modules.
 *
 * @author Thor Eivind and Mats (Master 2016 @ NTNU)
 */
public final class Application {

    private final String MAPLOCATION;
    private final Communication com;
    private final RobotController rc;
    private LinkedList<String> portList;
    private final MainGUI gui;
    private Simulator sim;
    private final NavigationController navigation;
    private final MappingController slam;
    private final MapGraphic worldMapGraphic;
    private final GridMap worldMap;
    private boolean simulatorActive = false;
    private boolean pause = false;

    /**
     * Constructor of the class Application
     */
    public Application() {
        Installer.generateSystemDependantLibraries();
        MainGUI.setLookAndFeel();
        this.MAPLOCATION = new File("maps\\big_map.txt").getAbsolutePath();
        this.rc = new RobotController();
        this.com = new Communication(rc);
        this.worldMap = new GridMap(2, 50, 50);
        this.worldMapGraphic = new MapGraphic(worldMap, rc);
        this.slam = new MappingController(rc, worldMap);
        this.navigation = new NavigationController(rc, this, worldMap);
        this.gui = new MainGUI(this);
        if (System.getProperty("os.name").startsWith("Windows")) {
            getPDFList();
        }

        getConnectedRobotList().addListener(new ListChangeListener<Robot>() {
            @Override
            public void onChanged(ListChangeListener.Change<? extends Robot> c) {
                c.next();
                if (c.wasAdded()) {
                    confirmHandshake(c.getAddedSubList().get(0).getId());
                    slam.addRobot(c.getAddedSubList().get(0).getName());
                    navigation.addRobot(c.getAddedSubList().get(0).getName(), c.getAddedSubList().get(0).getId());
                    /*}else if(c.wasReplaced()){
                    confirmHandshake(c.getAddedSubList().get(0).getId());
                     */
                }
            }
        });
    }

    /**
     * Opens a PDF using the default PDF viewer, the PDF should be in a manuals
     * folder thats next to the program path.
     *
     * @param name The name of the PDF to open, without ".pdf"
     */
    public void openPDF(String name) {
        if (Desktop.isDesktopSupported()) {
            try {
                String filename = "manuals\\" + name + ".pdf";
                File myFile = new File(filename);
                Desktop.getDesktop().open(myFile);
            } catch (IOException ex) {
                // no application registered for PDFs
            }
        }
    }

    /**
     * Method that returns a list of Strings containing the name all PDF's
     * inside the manuals folder that is in the same folder as the program
     *
     * @return LinkedList of Strings containing the name of all manuals
     */
    public LinkedList<String> getPDFList() {
        LinkedList<String> pdfList = new LinkedList<>();
        try {

            Files.walk(Paths.get("manuals\\")).forEach(filePath -> {
                if (Files.isRegularFile(filePath)) {
                    pdfList.addFirst(("" + filePath.getFileName()));
                }
            });
        } catch (IOException ex) {
            Logger.getLogger(Application.class.getName()).log(Level.SEVERE, null, ex);
        }
        return pdfList;
    }

    /**
     * Method that returns the world map
     *
     * @return the world map
     */
    public MapGraphic getWorldMap() {
        return worldMapGraphic;
    }

    //////////////////////////////////////////////////////////////////////
    //////////////////////////// INITIALIZERS ////////////////////////////
    //////////////////////////////////////////////////////////////////////
    /**
     * Method that starts the system for self-navigating autonomous robots
     */
    public void startSystem() {
        slam.start();
        navigation.start();
    }

    /**
     * Method that initiates the communication module, whether the simulator is
     * active or not
     *
     * @return boolean that tells if success or not
     */
    public boolean startCommunication() {
        if (!simulatorActive) {
            try {
                com.startCommunication(com.getComPort());
                com.startInboxReader();
            } catch (UnsupportedCommOperationException | PortInUseException | IOException | NoSuchPortException e) {
                return false;
            }
        }
        return true;
    }

    //////////////////////////////////////////////////////////////////////
    /////////////////////////// ROBOT HANDLING ///////////////////////////
    //////////////////////////////////////////////////////////////////////
    /**
     * Method that returns the available robot list
     *
     * @return the list
     */
    public ObservableList<String[]> getAvailableRobotList() {
        return rc.getAvailableRobotList();
    }

    /**
     * Method that returns the connected robot list
     *
     * @return the list
     */
    public ObservableList getConnectedRobotList() {
        return rc.getRobotList();
    }

    //////////////////////////////////////////////////////////////////////
    /////////////////////// SIMULATOR FUNCTIONALITY //////////////////////
    //////////////////////////////////////////////////////////////////////
    /**
     * Method that returns the simulator object
     *
     * @return the simulator
     */
    public Simulator getSim() {
        return sim;
    }

    /**
     * Method that sets the simulator object
     *
     * @param sim the simulator
     */
    public void setSim(Simulator sim) {
        this.sim = sim;
    }

    /**
     * Method that sets the simulator active or not
     *
     * @param active simulator mode
     */
    public void setSimulatorActive(boolean active) {
        if (active) {
            sim = new Simulator(com.getInbox());
            com.startInboxReader();
            sim.openGui();

        }
        simulatorActive = active;
    }

    /**
     * Returns the state of the simulator
     *
     * @return true if simulator mode
     */
    public boolean isSimulatorActive() {
        return simulatorActive;
    }

    //////////////////////////////////////////////////////////////////////
    /////////////////////// COMUNICATION COMMANDS ////////////////////////
    //////////////////////////////////////////////////////////////////////
    /**
     * Method that starts scanning for new robots
     */
    public void startScan() {
        if (!simulatorActive) {
            com.startScanForNewRobots();
        }
    }

    /**
     * Method that stops scanning for new robots
     */
    public void stopScan() {
        if (!simulatorActive) {
            com.stopScanForNewRobots();
        }
    }

    /**
     * Method that lists the found available robots
     */
    public void listAvailableRobots() {
        // Clears the old list
        rc.getAvailableRobotList().clear();
        if (!simulatorActive) {
            // If the simulator is not active, lists the found bluetooth robots
            com.listAvailableRobots();
        } else {
            for (String[] s : sim.listAvailableRobots()) {
                rc.addAvailableRobot(s);
            }
        }
    }

    /**
     * Method that initiates a connection to a specified robot
     *
     * @param robotID the robots id
     */
    public void connectToRobot(int robotID) {
        if (!simulatorActive) {
            com.connectToRobot(robotID);
        } else {
            sim.connectToRobot(robotID);
        }
    }

    /**
     * Method that cancels the connection to the specified robot
     *
     * @param robotID the robots id
     */
    public void disconnectRobot(int robotID) {
        if (!simulatorActive) {
            com.disconnectRobot(robotID);
        } else {
            sim.disconnectRobot(robotID);
        }
        String name = rc.getRobot(robotID).getName();
        rc.removeRobot(robotID);
        slam.removeRobot(name);
        navigation.removeRobot(name);
    }
    
    /**
     * Logger function
     */
    public void startLogging(int robotID){
        if (!simulatorActive) {
            com.sendStartDebugToRobot(robotID);
        }
    }

    public void stopLogging(int robotID){
        if (!simulatorActive) {
            com.sendStopDebugToRobot(robotID);
        }
    }    
    /**
     * Method that sends a command to a physical or simulated robot
     *
     * @param robotID the robots id
     * @param robotName the robots name
     * @param orientation the new orientation
     * @param distance the distance to go
     */
    public void writeCommandToRobot(int robotID, String robotName, double orientation, double distance) {
        //  newTime[robotID] = System.currentTimeMillis();
        if (!simulatorActive) {
            com.sendOrderToRobot(robotID, orientation, distance);
        } else {
            sim.setRobotCommand(robotID, orientation, distance);
        }
        rc.addStatus(robotName, "BUSY");
    }

    /**
     * Method that confirms that the server has received a handshake
     *
     * @param robotID the id of the robot.
     */
    public void confirmHandshake(int robotID) {
        if (!simulatorActive) {
            com.confirmHandshake(robotID);
        }
    }

    /**
     * Method that confirms that the robot is finished
     *
     * @param robotID the id of the robot.
     */
    public void confirmRobotFinished(int robotID) {
        if (!simulatorActive) {
            com.confirmRobotFinished(robotID);
        }
    }

    /**
     * Method that pauses the robot
     *
     * @param robotID the robots ID
     */
    public void pauseRobot(int robotID) {
        if (!simulatorActive) {
            com.pauseRobot(robotID);
        }
    }

    /**
     * Method that unpauses the robot
     *
     * @param robotID the robots ID
     */
    public void unPauseRobot(int robotID) {
        if (!simulatorActive) {
            com.unPauseRobot(robotID);
        }
    }

    //////////////////////////////////////////////////////
    /////////////// COMPORT FUNCTIONALITY ////////////////
    //////////////////////////////////////////////////////
    /**
     * Method that returns a list of available com ports
     *
     * @return the list
     */
    public LinkedList<String> getPortList() {
        if (!simulatorActive) {
            this.portList = com.listPorts();
            return portList;
        } else {
            return null;
        }
    }

    /**
     * Method that sets the current com port
     *
     * @param port the ports name
     */
    public void setComPort(String port) {
        com.setComPort(port);
    }

    //////////////////////////////////////////////////////
    /////////////////       MAIN       ///////////////////
    //////////////////////////////////////////////////////
    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        new Application();
    }

    /**
     * Pauses the navigation and slam
     */
    public void stopSystem() {
        navigation.pause();
        slam.pause();
    }

    /**
     * Resets the connected nRF51-dongle
     */
    public void resetDongle() {
        com.resetDongle();
    }

    /**
     * Method should be used before turning off the java program, this ensures
     * that the dongle is reset and disconnected before next use
     */
    public void turnOffProgram() {
        if (!simulatorActive) {
            try {
                System.out.println("Reseting dongle");
                resetDongle();
                Thread.sleep(100);
                System.out.println("Reset Complete");
            } catch (Exception e) {

            }
            try {
                System.out.println("Closing port");
                com.getSerialCommunication().closeComPort();
                System.out.println("Port closed");
            } catch (Exception e) {

            }
        }
    }
}
