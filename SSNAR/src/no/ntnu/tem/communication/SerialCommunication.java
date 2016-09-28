/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Thor Eivind Andersen and Mats Rødseth (Master 2016 @ NTNU)
 */
package no.ntnu.tem.communication;

import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.NoSuchPortException;
import gnu.io.PortInUseException;
import gnu.io.SerialPort;
import gnu.io.UnsupportedCommOperationException;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * This class represents the serial communication between the server and the
 * nRF51-dongle connected to it. It holds the communication parameters and
 * provides functionality for writing messages over serial to the dongle.
 *
 * @author Thor Eivind and Mats (Master 2016 @ NTNU)
 */
public class SerialCommunication extends Thread {

    private CommPortIdentifier portIdentifier;
    private CommPort commPort;
    private CommPort shdwCommPort;
    private SerialPort serialPort;
    private InputStream inStream;
    private OutputStream outStream;
    private StringBuilder sb = new StringBuilder();
    private String shdwPort;
    private boolean debug = false;
    private final Inbox inbox;

    /**
     * Constructor of the class Communication
     *
     * @param inbox The systems message inbox
     */
    public SerialCommunication(Inbox inbox) {
        this.inbox = inbox;
    }

    /**
     * Method for connecting to a given port
     *
     * @param portName the port to connect to
     * @throws gnu.io.UnsupportedCommOperationException .
     * @throws gnu.io.PortInUseException .
     * @throws java.io.IOException .
     * @throws gnu.io.NoSuchPortException .
     */
    public void connect(String portName) throws UnsupportedCommOperationException,
            PortInUseException, IOException, NoSuchPortException {
        disconnectCommPort(shdwCommPort);
        this.portIdentifier = CommPortIdentifier.getPortIdentifier(portName);
        if (portIdentifier.isCurrentlyOwned()) {
            System.out.println("Error: Port is currently in use");
        } else {
            this.commPort = portIdentifier.open(this.getClass().getName(), 2000);
            shdwCommPort = this.commPort;

            if (commPort instanceof SerialPort) {
                this.serialPort = (SerialPort) commPort;
                serialPort.setSerialPortParams(38400, SerialPort.DATABITS_8, SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);

                serialPort.setDTR(true);
                serialPort.setRTS(true);

                this.inStream = serialPort.getInputStream();
                this.outStream = serialPort.getOutputStream();

            } else {
                System.out.println("Error: Only serial ports are handled by this example.");
            }
        }
    }

    /**
     * Method that sends a message to the connected com-port
     *
     * @param message the message
     */
    public synchronized void send(String message) {
        try {
            byte[] data = message.getBytes();
            outStream.write(data);
            outStream.write(10);
            outStream.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Method that reads the in-stream and initiates parsing of received
     * messages
     */
    private void receiveMessages() {
        try {
            byte[] buffer = new byte[1024];
            int bytesRead;
            ByteArrayOutputStream output = new ByteArrayOutputStream();
            while (inStream.available() > 0) {
                bytesRead = inStream.read(buffer);
                output.write(buffer, 0, bytesRead);
                parseMessage(output.toByteArray());
                int i = 1;
            }
            output.close();
        } catch (IOException ex) {
            Logger.getLogger(SerialCommunication.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    /**
     * Stores the incoming messages in a StringBuilder. When a message is
     * complete it will be putted in Inbox
     *
     * @param mess The new incoming message
     */
    private void parseMessage(byte[] messageAsByteArray) {
        sb.append(new String(messageAsByteArray));
        String message = sb.toString();

        String[] strings = message.split("\\[");
        for (int j = 0; j < strings.length; j++) {
            if (debug) {
                System.out.println("Strings[" + j + "]: " + strings[j]);
            }
            if (endOfMessage(strings[j])) {
                // Fått slutten på en melding, tøm buffer
                sb = new StringBuilder();
                if (j == 0) {
                    inbox.putMessage("[" + strings[j]);
                }
                for (int i = 1; i < strings.length; i++) {
                    if (i <= j) {
                        if (debug) {
                            //System.out.println("MELDING: [" + strings[i]);
                        }
                        inbox.putMessage("[" + strings[i]);
                    } else {
                        sb.append("[" + strings[i]);
                    }
                }
            }
        }
    }

    /**
     * Checks whether the message ends with \n or not
     *
     * @param mess the message to check
     * @return true if message ends with \n
     */
    private boolean endOfMessage(String mess) {
        if (mess.endsWith("\n")) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void run() {
        super.run();
        while (true) {
            receiveMessages();
        }
    }

    /**
     * Disconnects the previous used com port
     *
     * @param shdwCommPort the port to disconnect
     */
    private void disconnectCommPort(CommPort shdwCommPort) {

        if (shdwCommPort != null) {
            new Thread() {
                @Override
                public void run() {
                    shdwCommPort.close();
                    if (debug) {
                        System.out.println("ComPort " + shdwCommPort.getName() + " closed.");
                    }
                }

            }.start();
        }
    }

    /**
     * Method that closes the current com-port
     */
    public void closeComPort() {
        disconnectCommPort(shdwCommPort);
    }
}
