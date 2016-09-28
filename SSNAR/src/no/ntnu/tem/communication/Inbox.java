/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Thor Eivind Andersen and Mats Rødseth (Master 2016 @ NTNU)
 */
package no.ntnu.tem.communication;

import java.util.LinkedList;
import java.util.ListIterator;

/**
 * This class represents the Inbox of the server application. The inbox is
 * implemented as a FIFO queue (Shared object).
 *
 * @author Thor Eivind and Mats (Master 2016 at NTNU)
 */
public class Inbox {

    private LinkedList messageList;
    private ListIterator itr;
    private boolean inboxEmpty = true;

    /**
     * Constructor of the class Inbox
     */
    public Inbox() {
        // Initializes the inbox-list (messageList) and its iterator (itr)
        this.messageList = new LinkedList<>();
        this.itr = messageList.listIterator();
    }

    /**
     * Method for putting a message into the inbox
     *
     * @param message Message to put into the inbox (JSONObject)
     */
    public synchronized void putMessage(String message) {
        messageList.add(message);
        inboxEmpty = false;
    }

    /**
     * Method that returns the current size of the inbox
     *
     * @return current inbox size
     */
    protected synchronized int getInboxSize() {
        return messageList.size();
    }

    /**
     * Method that returns the first message in the inbox
     *
     * @return The first message in the FIFO inbox
     */
    protected synchronized String getMessage() {
        itr = messageList.listIterator();
        if (itr.hasNext()) {
            String message = (String) itr.next();
            itr.remove();
            return message;
        } else {
            inboxEmpty = true;
            return null;
        }
    }
}
