/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Thor Eivind Andersen and Mats Rødseth (Master 2016 @ NTNU)
 */
package no.ntnu.tem.communication;

/**
 * This class provides the naming convention for the different parameters used
 * in the MessageHandler class
 *
 * @author Thor Eivind and Mats (Master 2016 @ NTNU)
 */
public interface Language {

    // Naming convention for message content
    public final static int NUMBEROFSENSORS = 4; // det her bør vel ligge i handshaken?
    public final static String UPDATE = "U";
    public final static String HANDSHAKE = "H";
    public final static String STATUS = "S";
    public final static int HANDSHAKE_LENGTH = 15;
    public final static int UPDATE_LENGTH = 9;
    public final static int STATUS_LENGTH = 2;

    // Where to find the different content in the messages
    //// UPDATE
    public final static int MESSAGETYPE = 0;
    public final static int UPDATE_XPOS = 1;
    public final static int UPDATE_YPOS = 2;
    public final static int UPDATE_THETA = 3;
    public final static int UPDATE_TOWER_HEADING = 4;
    public final static int UPDATE_SENSOR_START = 5;
    public final static int UPDATE_SENSOR_END = 8;
    //// HANDSHAKE
    public final static int HANDSHAKE_ROBOTWIDTH = 1;
    public final static int HANDSHAKE_ROBOTLENGTH = 2;
    public final static int HANDSHAKE_TOWEROFFSET_Y = 3;
    public final static int HANDSHAKE_TOWEROFFSET_X = 4;
    public final static int HANDSHAKE_AXLEOFFSET = 5;
    public final static int HANDSHAKE_SENSOROFFSET_START = 6;
    public final static int HANDSHAKE_SENSOROFFSET_END = 9;
    public final static int HANDSHAKE_IRHEADING_START = 10;
    public final static int HANDSHAKE_IRHEADING_END = 13;
    public final static int HANDSHAKE_MESSAGEDEADLINE = 14;
    //// STATUS
    public final static int STATUS_START = 1;
    public final static String STATUS_IDLE = "IDL";
    public final static String STATUS_BUSY = "BUSY";
    public final static String STATUS_HIT = "HIT";
    
    //// DEBUG
    public final static String DEBUG = "D";
    
    //// MAP
    public final static String MAP = "M";
    
    //// VALUES
    public final static int IR_MAX_VALUE = 90;
    public final static int IR_MIN_VALUE = 0;
    public final static int TOWER_HEADING_MAX = 100;
    public final static int TOWER_HEADING_MIN = 0;

}
