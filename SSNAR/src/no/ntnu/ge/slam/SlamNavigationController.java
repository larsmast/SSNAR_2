/**
 * This code is written as part of a Master Thesis
 * the spring of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 */
package no.ntnu.ge.slam;

import no.ntnu.et.simulator.SlamRobot;

/**
 *
 * @author geirhei
 */
public class SlamNavigationController extends Thread {
    private boolean paused = false;
    SlamRobot robot;
    int[] command;
    
    public SlamNavigationController(SlamRobot robot) {
        this.robot = robot;
        command = new int[] {0, 0};
    }
    
    @Override
    public void run() {
        setName("Slam navigation controller");
        while (true) {
            try {
                Thread.sleep(5000);
            } catch (InterruptedException e) {
                e.printStackTrace();
                break;
            }
            if (paused) {
                continue;
            }
            
            //command = findCommandToTarget(0, 0);
            if (!robot.isBusy()) {
                command = findCommandToTarget(90, 10);
            }
            robot.setTarget(90, 10);
            robot.setBusy(true);
        }
    }
    
    private int[] findCommandToTarget(int rotation, int distance) {
        int[] cmd = {rotation, distance};
        return cmd;
    }
}
