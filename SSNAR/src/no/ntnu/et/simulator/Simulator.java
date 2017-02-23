/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.simulator;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;
import javax.swing.JFrame;
import no.ntnu.ge.slam.SlamMappingController;
import no.ntnu.ge.slam.SlamNavigationController;
import no.ntnu.tem.communication.Inbox;

/**
 * This class contains both the interface and the control system of the
 * simulator. This is the main class in the simulator package.
 *
 * @author Eirik Thon
 */
public class Simulator {
    private final SimWorld world;
    private SimulatorGUI gui;
    private boolean estimateNoiseEnabled;
    private boolean sensorNoiseEnabled;
    HashMap<Integer, RobotHandler> robotHandlers;
    private double simulationSpeed;
    private Inbox inbox;
    private int mode;
    private HashMap<Integer, String> idNameMapping;

    /**
     * Constructor. Creates an instance of the Simulator class with a number of
     * robots given by the length of "robotNames" and map from the file
     * specified by "mapPath"
     *
     * @param robotNames String[]
     * @param mapPath String
     */
    public Simulator(Inbox inbox) {
        simulationSpeed = 1;
        world = new SimWorld();
        File mapFolder = new File("maps");
        File[] allMaps = mapFolder.listFiles();
        if(allMaps.length == 0){
            System.out.println("No map files found");
        }
        String mapPath = allMaps[0].getAbsolutePath();
        world.initMap(mapPath);
        robotHandlers = new HashMap<Integer, RobotHandler>();
        estimateNoiseEnabled = true;
        sensorNoiseEnabled = true;
        this.mode = mode;
        javax.swing.SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                initializeGUI();
            }
        });
        this.inbox = inbox;
    }

    double getSimulationSpeed() {
        return simulationSpeed;
    }

    void setSimulationSpeed(double newSpeed) {
        simulationSpeed = (double) newSpeed / 100;
    }

    public void openGui() {
        javax.swing.SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                gui.setVisible(true);
            }
        });
    }

    public void closeGui() {
        javax.swing.SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                gui.setVisible(false);
            }
        });
    }

    public static void main(String[] args) throws InterruptedException {
        // Create and run an instance if the Simulator class
    }

    /**
     * Stops the simulator, and closes the window
     */
    public void stop() {
        // Uses the closing routine defined in initializeGUI
        for (HashMap.Entry<Integer, RobotHandler> entry : robotHandlers.entrySet()) {
            entry.getValue().interrupt();
        }
        javax.swing.SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                gui.dispose();
            }
        });
    }

    public void connectToRobot(int id) {
        if (!robotHandlers.containsKey(id)) {
            SimRobot robot = world.getRobot(id);
            RobotHandler robotHandler;
            if (id == 0) {
                robotHandler = this.new SlamRobotHandler((SlamRobot)robot);
            } else {
                robotHandler = this.new SimRobotHandler((BasicRobot)robot);
            }
            robotHandler.setName("Robot " + Integer.toString(id));
            robotHandlers.put(id, robotHandler);
            robotHandlers.get(id).start();
        }
    }

    public void disconnectRobot(int id) {
        if (robotHandlers.containsKey(id)) {
            robotHandlers.get(id).interrupt();
            robotHandlers.remove(id);
        }
    }

    public ArrayList<String[]> listAvailableRobots() {
        ArrayList<String[]> availableRobots = new ArrayList<String[]>();
        ArrayList<Integer> robotIDs = world.getRobotIDs();
        for (int i = 0; i < robotIDs.size(); i++) {
            /*
            if(robotHandlers.containsKey(id)){
                continue;
            }*/
            String name = world.getRobot(robotIDs.get(i)).getName();
            String[] robotId = {"" + robotIDs.get(i), name};
            availableRobots.add(robotId);
        }
        return availableRobots;
    }

    /**
     * Turns estimation error on/off
     */
    void flipEstimateNoiseEnabled() {
        estimateNoiseEnabled = !estimateNoiseEnabled;
    }

    /**
     * Turns sensor error on/off
     */
    void flipSensorNoiseEnabled() {
        sensorNoiseEnabled = !sensorNoiseEnabled;
    }

    /**
     * Gives the command ("rotation", "angle") to the robot with name "name"
     *
     * @param name String
     * @param rotation double
     * @param distance double
     */
    public void setRobotCommand(int id, double rotation, double distance) {
        world.getRobot(id).setTarget(rotation, distance);
    }

    public void pauseRobot(int robotId) {
        robotHandlers.get(robotId).pause();
    }

    public void unpauseRobot(int robotId) {
        robotHandlers.get(robotId).unpause();
    }

    /**
     * Initializes the graphical user interface
     */
    private void initializeGUI() {
        gui = new SimulatorGUI(this, world);
        gui.setTitle("Simulator");
        gui.setVisible(false);
        gui.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);

        if (!sensorNoiseEnabled) {
            gui.checkMeasurementErrorOff();
        }

        if (!estimateNoiseEnabled) {
            gui.checkEstimationErrorOff();
        }
    }

    /**
    * Controller object for a robot of type BasicRobot. This class contains
 a BasicRobot object and is responsible for making that robot run.
    */
    private class SimRobotHandler extends RobotHandler {
        final private BasicRobot myRobot;
        final private String myName;
        final private int myID;
        private double estimateNoise;
        private double sensorNoise;
        private final Random noiseGenerator;
        private boolean paused;

        /**
         * Constructor
         *
         * @param robot BasicRobot
         */
        public SimRobotHandler(BasicRobot robot) {
            myRobot = robot;
            myName = robot.getName();
            myID = robot.getId();
            noiseGenerator = new Random();
        }

        /**
         * A continuous loop that calls methods of the robot in a specific way
         * in order to generate the robot behavior.
         */
        @Override
        public void run() {
            int counter = 0;
            String content = myRobot.getHandShakeMessage();
            String dongleID = "[" + myID + "]:" + myName + ":";
            String handshake = dongleID + content;
            inbox.putMessage(handshake);
            while (true) {
                // Wait between each loop
                try {
                    Thread.sleep((int) (10 / simulationSpeed));
                } catch (InterruptedException e) {
                    break;
                }
                if (isPaused()) {
                    continue;
                }

                // Move robot
                if (estimateNoiseEnabled) {
                    estimateNoise = noiseGenerator.nextGaussian() * 0.1;
                } else {
                    estimateNoise = 0;
                }
                if (myRobot.moveRobot(estimateNoise) == true) {
                    String updateMsg = "{S,IDL}\n";
                    String dongleSim = "[" + myID + "]:" + myName + ":";
                    inbox.putMessage(dongleSim + updateMsg);
                }
                myRobot.turnTower();
                
                // Measure
                if (counter > 19) { // Every 200 ms ( Counter is increased every 10 ms )
                    if (sensorNoiseEnabled) {
                        sensorNoise = noiseGenerator.nextGaussian() * 2;
                    } else {
                        sensorNoise = 0;
                    }
                    myRobot.measureIR(sensorNoise);
                    int[] update = myRobot.createMeasurement();
                    String updateMsg = BasicRobot.generateUpdate(update[0], update[1], update[2], update[3], update[4], update[5], update[6], update[7]);
                    String dongleSim = "[" + myID + "]:" + myName + ":";
                    inbox.putMessage(dongleSim + updateMsg);
                    counter = 0;
                }
                counter++;
            }
        }
    }
    
    /**
    * Controller object for a robot of type SlamRobot. This class contains
    * a SlamRobot object and is responsible for making that robot run.
    */
    private class SlamRobotHandler extends RobotHandler {
        final private SlamRobot myRobot;
        final private String myName;
        final private int myID;
        private double estimateNoise;
        private double sensorNoise;
        private final Random noiseGenerator;
        SlamMappingController mapping;
        SlamNavigationController navigation;
        
        /**
         * Constructor
         *
         * @param robot SlamRobot
         */
        public SlamRobotHandler(SlamRobot robot) {
            myRobot = robot;
            myName = robot.getName();
            myID = robot.getId();
            noiseGenerator = new Random();
        }
        
        /**
         * A continuous loop that calls methods of the robot in a specific way
         * in order to generate the robot behavior.
         */
        @Override
        public void run() {
            mapping = new SlamMappingController(myRobot, inbox);
            mapping.setName("SlamMappingController");
            mapping.start();
            navigation = new SlamNavigationController(myRobot);
            navigation.setName("SlamNavigationController");
            navigation.start();
            
            int counter = 0;
            String content = myRobot.getHandShakeMessage();
            String dongleID = "[" + myID + "]:" + myName + ":";
            String handshake = dongleID + content;
            inbox.putMessage(handshake);
            while (true) {
                // Wait between each loop
                try {
                    Thread.sleep((int) (10 / simulationSpeed));
                } catch (InterruptedException e) {
                    break;
                }
                if (isPaused()) {
                    continue;
                }

                // Move robot
                if (estimateNoiseEnabled) {
                    estimateNoise = noiseGenerator.nextGaussian() * 0.1;
                } else {
                    estimateNoise = 0;
                }
                if (myRobot.moveRobot(estimateNoise) == true) {
                    String updateMsg = "{S,IDL}\n";
                    String dongleSim = "[" + myID + "]:" + myName + ":";
                    inbox.putMessage(dongleSim + updateMsg);
                    myRobot.setBusy(false);
                }
                myRobot.turnTower();
                
                // Measure
                if (counter > 19) { // Every 200 ms ( Counter is increased every 10 ms )
                    if (sensorNoiseEnabled) {
                        sensorNoise = noiseGenerator.nextGaussian() * 2;
                    } else {
                        sensorNoise = 0;
                    }
                    myRobot.measureIR(sensorNoise);
                    myRobot.updateIrHeading();
                    int[] update = myRobot.createMeasurement();
                    myRobot.addUpdate(update); // add update to updateQueue -> SlamMappingController
                    String updateMsg = BasicRobot.generateUpdate(update[0], update[1], update[2], update[3], update[4], update[5], update[6], update[7]);
                    String dongleSim = "[" + myID + "]:" + myName + ":";
                    inbox.putMessage(dongleSim + updateMsg);
                    counter = 0;
                }
                counter++;
            }
        }
    }
}
