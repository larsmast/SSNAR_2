/**
 * This code is written as part of a Master Thesis
 * the spring of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 */
package no.ntnu.ge.slam;

import java.util.concurrent.LinkedBlockingQueue;
import no.ntnu.et.simulator.SlamRobot;
import no.ntnu.tem.communication.Inbox;

/**
 *
 * @author geirhei
 */
public class SlamMappingController extends Thread {
    private final int cellSize = 2;
    private SlamRobot robot;
    private Inbox inbox;
    private int[][] mapWindow;
    private boolean paused;
    private LinkedBlockingQueue<int[]> measurementQueue;
    private int[] currentMeasurement;
    
    public SlamMappingController(SlamRobot robot, Inbox inbox) {
        this.robot = robot;
        this.inbox = inbox;
        mapWindow = robot.getMapWindow();
        measurementQueue = robot.getMeasurementQueue();
        currentMeasurement = new int[20];
    }
    
    @Override
    public void start(){
        if(!isAlive()){
            super.start();
        }
        paused = false;
    }

    /**
     * Pauses the mapping
     */
    public void pause(){
        paused = true;
    }
    
    /**
     * Returns if the mapping is running or paused
     * @return 
     */
    public boolean isRunning(){
        return !paused;
    }
    
    @Override
    public void run() {
        while (true) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
                break;
            }
            if (paused){
                continue;
            }
            
            currentMeasurement = measurementQueue.poll();
            if (currentMeasurement == null) {
                System.out.println("currentMeasurement = null");
                continue;
            }
            
        }
    }
}
