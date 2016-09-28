/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Thor Eivind Andersen and Mats Rødseth (Master 2016 @ NTNU)
 */
package no.ntnu.tem.communication;

import java.util.Arrays;

/**
 * This class provides functionality for interpreting received messages.
 *
 * @author Thor Eivind and Mats (Master 2016 @ NTNU)
 */
public class MessageHandler implements Language {

    /**
     * Method that returns message type (handshake/update)
     *
     * @param str message to interpret
     * @return message type (handshake or update)
     * @throws no.ntnu.tem.communication.MessageHandler.MessageCorruptException
     * if str is not structured properly
     */
    public static String getMessageType(String str) throws MessageCorruptException {
        try {
            return (String) str.split(",")[MESSAGETYPE];
        } catch (Exception e) {
            throw new MessageCorruptException();
        }
    }

    /**
     * Method that checks if the number of parameters in the content is correct
     *
     * @param str message
     * @param messagetype type of message (handshake, update, status)
     * @throws no.ntnu.tem.communication.MessageHandler.MessageCorruptException
     */
    public static void checkIfMessageIsIntact(String str, String messagetype) throws MessageCorruptException {
        try {
            switch (messagetype) {
                case HANDSHAKE:
                    if (str.split(",").length != HANDSHAKE_LENGTH) {
                        throw new MessageCorruptException();
                    }
                    break;
                case UPDATE:
                    if (str.split(",").length != UPDATE_LENGTH) {
                        throw new MessageCorruptException();
                    }
                    break;
                case STATUS:
                    if (str.split(",").length != STATUS_LENGTH) {
                        throw new MessageCorruptException();
                    }
                    break;
            }
        } catch (Exception e) {
            throw new MessageCorruptException();
        }
    }

    /**
     * Method that returns the IR tower angle (in update message)
     *
     * @param update message to interpret
     * @return the angle of the IR tower
     * @throws no.ntnu.tem.communication.MessageHandler.MessageCorruptException
     * if update is not structured properly
     * @throws no.ntnu.tem.communication.MessageHandler.ValueCorruptException if
     * some of the values are out of bounds
     */
    public static int getRobotTowerAngle(String update) throws MessageCorruptException, ValueCorruptException {
        try {
            String[] content = update.split(",");
            int towerAngle = Integer.parseInt(content[UPDATE_TOWER_HEADING]);
            if (towerAngle < TOWER_HEADING_MIN || towerAngle > TOWER_HEADING_MAX) {
                throw new ValueCorruptException();
            }
            return towerAngle;
        } catch (NumberFormatException e) {
            throw new MessageCorruptException();
        }
    }

    /**
     * Method that returns the IR headings (in handshake messages)
     *
     * @param handshake message to interpret
     * @return an array of all IR headings, can be used to calculate ir
     * spreading
     * @throws no.ntnu.tem.communication.MessageHandler.MessageCorruptException
     * if handshake is not structured properly
     */
    public static int[] getRobotIRHeading(String handshake) throws MessageCorruptException {
        try {
            String[] content = handshake.split(",");
            int[] data = new int[NUMBEROFSENSORS];
            content = Arrays.copyOfRange(content, 10, 14);
            for (int i = 0; i < content.length; i++) {
                data[i] = Integer.parseInt(content[i]);
            }
            return data;
        } catch (Exception e) {
            throw new MessageCorruptException();
        }
    }

    /**
     * Method that returns the data gathered by the ir-sensors. Used only during
     * update.
     *
     * @param update message to interpret
     * @return array containing the sensor data
     * @throws no.ntnu.tem.communication.MessageHandler.MessageCorruptException
     * if update is not structured properly
     * @throws no.ntnu.tem.communication.MessageHandler.ValueCorruptException if
     * some of the values are out of bounds
     */
    public static int[] getRobotIRData(String update) throws MessageCorruptException, ValueCorruptException {
        try {
            String[] content = update.split(",");
            int[] data = new int[NUMBEROFSENSORS];
            content = Arrays.copyOfRange(content, UPDATE_SENSOR_START, UPDATE_SENSOR_END + 1);
            for (int i = 0; i < content.length; i++) {
                data[i] = Integer.parseInt(content[i]);
                if (data[i] < IR_MIN_VALUE || data[i] > IR_MAX_VALUE) {
                    throw new ValueCorruptException();
                }
            }
            return data;
        } catch (NumberFormatException e) {
            throw new MessageCorruptException();
        }
    }

    /**
     * Method that returns the robots position. Used only during Update
     *
     * @param update message to interpret
     * @return array containing the x and y coordinate of the robot
     * @throws no.ntnu.tem.communication.MessageHandler.MessageCorruptException
     * if update is not structured properly
     */
    public static int[] getRobotPositionData(String update) throws MessageCorruptException {
        try {
            String[] content = update.split(",");
            return new int[]{Integer.parseInt(content[UPDATE_XPOS]), Integer.parseInt(content[UPDATE_YPOS])};
        } catch (Exception e) {
            throw new MessageCorruptException();
        }
    }

    /**
     * Method that returns the robots orientation. Used only during Update
     *
     * @param update message to interpret
     * @return the robots orientation
     * @throws no.ntnu.tem.communication.MessageHandler.MessageCorruptException
     * if update is not structured properly
     */
    public static int getOrientationData(String update) throws MessageCorruptException {
        try {
            String[] content = update.split(",");
            int data = Integer.parseInt(content[UPDATE_THETA]);
            return data;
        } catch (Exception e) {
            throw new MessageCorruptException();
        }
    }

    /**
     * Method that returns the robots physical width. Used only during Handshake
     *
     * @param handshake message to interpret
     * @return the robot width
     * @throws no.ntnu.tem.communication.MessageHandler.MessageCorruptException
     * if handshake is not structured properly
     */
    public static int getRobotWidth(String handshake) throws MessageCorruptException {
        try {
            String[] content = handshake.split(",");
            int data = Integer.parseInt(content[HANDSHAKE_ROBOTWIDTH]);
            return data;
        } catch (Exception e) {
            throw new MessageCorruptException();
        }
    }

    /**
     * Method that returns the robots physical length. Used only during
     * Handshake
     *
     * @param handshake message to interpret
     * @return the robot length
     * @throws no.ntnu.tem.communication.MessageHandler.MessageCorruptException
     * if handshake is not structured properly
     */
    public static int getRobotLength(String handshake) throws MessageCorruptException {
        try {
            String[] content = handshake.split(",");
            int data = Integer.parseInt(content[HANDSHAKE_ROBOTLENGTH]);
            return data;
        } catch (Exception e) {
            throw new MessageCorruptException();
        }
    }

    /**
     * Method that returns the tower offset [a,b]: a-lengthwise, b-crosswise.
     * Used during Handshake.
     *
     * @param handshake message to interpret
     * @return array containing the tower offset
     * @throws no.ntnu.tem.communication.MessageHandler.MessageCorruptException
     * if handshake is not structured properly
     */
    public static int[] getTowerOffset(String handshake) throws MessageCorruptException {
        try {
            String[] content = handshake.split(",");
            return new int[]{Integer.parseInt(content[HANDSHAKE_TOWEROFFSET_Y]), Integer.parseInt(content[HANDSHAKE_TOWEROFFSET_X])};
        } catch (Exception e) {
            throw new MessageCorruptException();
        }
    }

    /**
     * Method that returns the axle offset lengthwise. Used during Handshake
     *
     * @param handshake message to interpret
     * @return the axle offset
     * @throws no.ntnu.tem.communication.MessageHandler.MessageCorruptException
     * if handshake is not structured properly
     */
    public static int getAxleOffset(String handshake) throws MessageCorruptException {
        try {
            String[] content = handshake.split(",");
            int data = Integer.parseInt(content[HANDSHAKE_AXLEOFFSET]);
            return data;
        } catch (Exception e) {
            throw new MessageCorruptException();
        }

    }

    /**
     * Method that returns the sensor offset from center of the tower (radius).
     * Used during Handshake
     *
     * @param handshake message to interpret
     * @return array containing the sensor offset
     * @throws no.ntnu.tem.communication.MessageHandler.MessageCorruptException
     * if handshake is not structured properly
     */
    public static int[] getSensorOffset(String handshake) throws MessageCorruptException {
        try {
            String[] content = handshake.split(",");
            int[] data = new int[NUMBEROFSENSORS];
            content = Arrays.copyOfRange(content, HANDSHAKE_SENSOROFFSET_START, HANDSHAKE_SENSOROFFSET_END + 1);
            for (int i = 0; i < content.length; i++) {
                data[i] = Integer.parseInt(content[i]);
            }
            return data;
        } catch (Exception e) {
            throw new MessageCorruptException();
        }
    }

    /**
     * Method that returns the robots message deadline. Used during handshake
     *
     * @param handshake message to interpret
     * @return the message deadline (ms)
     * @throws no.ntnu.tem.communication.MessageHandler.MessageCorruptException
     * if handshake is not structured properly
     */
    public static int getMessageDeadline(String handshake) throws MessageCorruptException {
        try {
            String[] content = handshake.split(",");
            int data = Integer.parseInt(content[HANDSHAKE_MESSAGEDEADLINE]);
            return data;
        } catch (Exception e) {
            e.printStackTrace();
            throw new MessageCorruptException();
        }
    }

    /**
     * Method that returns the robots status. Used during status messages
     *
     * @param status
     * @return
     * @throws no.ntnu.tem.communication.MessageHandler.MessageCorruptException
     */
    public static String getRobotStatus(String status) throws MessageCorruptException {
        try {
            String[] content = status.split(",");
            return content[STATUS_START];
        } catch (Exception e) {
            throw new MessageCorruptException();
        }
    }

    /**
     * Method that wraps content into a message
     *
     * @param orientation the robots orientation
     * @param distance the robots distance to travel
     * @return wrapped message
     */
    public static String wrapRobotOrder(double orientation, double distance) {
        String message = "{" + UPDATE + "," + orientation + "," + distance + "}";
        return message;
    }

    /**
     * Method that wraps a "Handshake confirmation" command into a message
     *
     * @param statusType 1 if "Handshake confirmed, 2 if "Robot finished"
     * @return wrapped message
     */
    public static String wrapStatus(String statusType) {
        String message = "{" + STATUS + "," + statusType + "}";
        return message;
    }

    /**
     * Method that wraps a "Switch robot" command into a message
     *
     * @param robotID the robot ID
     * @return wrapped message
     */
    public static String wrapSwitchToRobot(int robotID) {
        String message = "switch " + robotID;
        return message;
    }

    /**
     * Method that wraps a "Connect to robot" command into a message
     *
     * @param robotID the robot ID
     * @return wrapped message
     */
    public static String wrapConnectToRobot(int robotID) {
        String message = "conn " + robotID;
        return message;
    }

    /**
     * Method that wraps a "Disconnect robot" command into a message
     *
     * @param robotID the robot ID
     * @return wrapped message
     */
    public static String wrapDisconnectRobot(int robotID) {
        String message = "drop " + robotID;
        return message;
    }

    /**
     * Exception thrown when the message is corrupt
     */
    public static class MessageCorruptException extends Exception {

        public MessageCorruptException() {
        }
    }

    /**
     * Exception thrown when a value is outside its range
     */
    public static class ValueCorruptException extends Exception {

        public ValueCorruptException() {
        }
    }
}
