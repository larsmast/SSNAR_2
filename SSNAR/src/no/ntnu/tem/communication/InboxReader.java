/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Thor Eivind Andersen and Mats Rødseth (Master 2016 @ NTNU)
 */
package no.ntnu.tem.communication;

import java.util.HashMap;
import java.util.logging.Level;
import java.util.logging.Logger;
import no.ntnu.tem.application.RobotController;
import java.io.FileWriter;
import java.io.IOException;

/**
 * This class retrieves the messages from the inbox and interprets them using
 * the class MessageHandler. Furthermore it pushes the information to the
 * RobotController and thus to the rest of the application.
 *
 * @author Thor Eivind and Mats (Master 2016 at NTNU)
 */
public class InboxReader extends Thread implements Language {

    private final RobotController rc;
    private final Inbox inbox;
    private final boolean debug = false;

    private final HashMap<Integer, String> messageList;

    /**
     * Constructor of the class InboxReader
     *
     * @param inbox the systems Inbox
     * @param rc the systems RobotController
     */
    public InboxReader(Inbox inbox, RobotController rc) {
        this.setName("InboxReader");
        this.rc = rc;
        this.inbox = inbox;
        this.messageList = new HashMap<>();
    }

    /**
     * Method that retrieves a message from the inbox and initiates the
     * interpreting (if the message is complete)
     */
    private void readMessage() {
        if (inbox.getInboxSize() > 0) {
            String message = inbox.getMessage();
            try {
                // Decode message
                if (debug) {
                    System.out.println("ReadMessage: " + message);
                }
                int robotID = Integer.parseInt(message.substring(1, 2));
                //NOTE:
                //since this only takes one char, only 10 robots can be
                //added to the system at the same time. The reason why
                //we only check one char is because there is a limit in 
                //the code on NRF51, this can be fixed by changing the
                //number to char converter in the messages on NRF51, the
                //reason why this is not done is because its faster and 
                //easier to just do it in one char and we currently are
                //only testing with 2 maximum 3 robots at the same time.
                //However this puts a stop at 10 robots in simulator..

                //Append to correct id
                //Check if content is complete
                String name = message.split(":")[1];
                String tmpContent = message.split(":")[2];

                String content = messageMerger(robotID, tmpContent);

                // If the message is not complete, content = null, and the inboxreader
                // thread jumps out of the loop and re-runs readMessage
                if (content == null) {

                    return; //incomplete message,
                }
                //Messagehandler
                if (debug) {
                    System.out.println("Hello, i'm " + name + ", i'm number " + robotID + " and my content is: ");
                }

                interpretContent(name, robotID, content);
            } catch (Exception e) {
                //System.out.println("Generell error i melding: " + message);
            }
        } else {
            try {
                Thread.sleep(10);
            } catch (InterruptedException ex) {
                //Logger.getLogger(InboxReader.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
    }

    /**
     * Method that interprets the new message
     *
     * @param name the name of the sender
     * @param robotID the senders id
     * @param content the content of the message
     */
    private void interpretContent(String name, int robotID, String content) {
        try {
            content = content.replace("{", "");
            content = content.replace("}", "");
            content = content.replace("\n", "");
            if (debug) {
                System.out.println(content);
            }

            if (name.equalsIgnoreCase("nrf")) {
                String code = content.split("-")[0];
                switch (code) {
                    case "new":
                        robotID = Integer.parseInt(content.split("-")[1]);
                        name = content.split("-")[2];
                        if (debug) {
                            System.out.println("ID: " + robotID + ", NAME: " + name);
                        }
                        rc.addAvailableRobot(robotID, name);
                        break;
                    case "connected to":
                        // TODO (?)
                        break;
                }
            } else {
                String messagetype = MessageHandler.getMessageType(content);

                switch (messagetype) {
                    case HANDSHAKE:
                        if (content.split(",").length != HANDSHAKE_LENGTH) {
                            //    System.out.println("Feil, size: "+content.split(",").length +" content: " +content);
                            break;
                        }
                        if (debug) {
                            System.out.println("CASE: HANDSHAKE");
                            System.out.println("Message size: ");
                        }

                        int width = MessageHandler.getRobotWidth(content);
                        int length = MessageHandler.getRobotLength(content);
                        int axleOffset = MessageHandler.getAxleOffset(content);
                        int messageDeadline = MessageHandler.getMessageDeadline(content);
                        int[] towerOffset = MessageHandler.getTowerOffset(content);
                        int[] sensorOffset = MessageHandler.getSensorOffset(content);
                        int[] irHeading = MessageHandler.getRobotIRHeading(content);
                        doHandshake(robotID, name, width, length, axleOffset, messageDeadline, towerOffset, sensorOffset, irHeading);

                        break;

                    case UPDATE: //UPDATE
                        if (content.split(",").length != UPDATE_LENGTH) {
                            //  System.out.println("Feil, size: "+content.split(",").length +" content: " +content);
                            break;
                        }
                        if (debug) {
                            System.out.println("CASE: UPDATE");
                        }
                        int orientation = MessageHandler.getOrientationData(content);
                        int[] position = MessageHandler.getRobotPositionData(content);
                        int towerAngle = MessageHandler.getRobotTowerAngle(content);
                        int[] irData = MessageHandler.getRobotIRData(content);
                        doUpdate(name, orientation, position, towerAngle, irData);
                        break;

                    case STATUS: //STATUS
                        if (debug) {
                            System.out.println("CASE: STATUS");
                        }
                        String status = MessageHandler.getRobotStatus(content);
                        if (debug) {
                            System.out.println("STATUS: " + status);
                        }
                        doStatusUpdate(name, status);
                        break;
                    // Added Henrik Logfunction
                    case DEBUG:
                        if (debug) {
                            System.out.println("CASE: DEBUG");
                        }
                        try (FileWriter fw = new FileWriter("LOG.txt", true)) {
                            fw.write(content);
                            fw.write("\n");
                            fw.flush();
                        } catch (IOException e) {
                            System.err.println("IOexception: " + e.getMessage());
                        }
                        System.out.println(content);
                        break;
                        
                    case MAP:
                        if (debug) {
                            System.out.println("CASE: MAP");
                        }
                        break;
                }

            }

        } catch (MessageHandler.MessageCorruptException ex) {
            rc.getRobot(robotID).addMessageCorruptCount();
            //  Logger.getLogger(InboxReader.class.getName()).log(Level.SEVERE, null, ex);
        } catch (MessageHandler.ValueCorruptException ex) {
            rc.getRobot(robotID).addValueCorruptCount();
            //   Logger.getLogger(InboxReader.class.getName()).log(Level.SEVERE, null, ex);
        }

    }

    /**
     * Method that retrieves a message related to the specified robot id from
     * the inbox
     *
     * @param id robot id
     * @return
     */
    private String findMessageByRobotID(int id) {
        if (messageList.containsKey(id)) {
            return messageList.get(id);
        } else {
            return "";
        }
    }

    /**
     * Method that puts complete messages into the messageList
     *
     * @param robotID the robots id
     * @param content the message content
     * @return null if the message is not complete
     */
    private String messageMerger(int robotID, String content) {
        String mergedContent = findMessageByRobotID(robotID) + content;
        if (debug) {
            System.out.println("Merged" + mergedContent);
        }
        if (mergedContent.startsWith("{")) {
            if (mergedContent.endsWith("\n")) {
                messageList.put(robotID, "");
                return mergedContent;
            } else {
                messageList.put(robotID, mergedContent);
                return null;
            }
        }
        return null;
    }

    @Override
    public void run() {
        super.run(); //To change body of generated methods, choose Tools | Templates.
        while (true) {
            readMessage();
        }
    }

    /**
     * Method for adding a new robot to the system after receiving a handshake
     *
     * @param name The robots name
     * @param width The robots physical width (cm)
     * @param length The robots physical length (cm)
     * @param axleOffset Axle offset lengthwise
     * @param messageDeadline The robots message deadline (min update rate[ms])
     * @param towerOffset Tower offset [a,b] a-lengthwise, b-crosswise
     * @param sensorOffset Sensor offset [a,b,c,d,..] from centre (radius)
     * @param irHeading IR heading relative to 0 deg (straight forward)
     */
    private void doHandshake(int robotID, String name, int width, int length,
            int axleOffset, int messageDeadline, int[] towerOffset, int[] sensorOffset,
            int[] irHeading) {
        rc.addRobot(robotID, name, width, length, messageDeadline, axleOffset,
                towerOffset, sensorOffset, irHeading);
    }

    /**
     * Method for updating one of the robots in the system
     *
     * @param name The robots name
     * @param orientation The robots orientation
     * @param position The robots position (x,y)
     * @param irHeading The IR-sensors heading
     * @param irData Data gathered from IR-sensors
     */
    private void doUpdate(String name, int orientation, int[] position,
            int irHeading, int[] irData) {
        rc.addMeasurment(name, orientation, position, irHeading, irData);
    }

    /**
     * Method for updating the status to one of the robots in the system
     *
     * @param name
     * @param status
     */
    private void doStatusUpdate(String name, String status) {
        rc.addStatus(name, status);
    }

}
