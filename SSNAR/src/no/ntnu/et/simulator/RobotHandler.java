/**
 * This code is written as part of a Master Thesis
 * the spring of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 */
package no.ntnu.et.simulator;

/**
 * An abstract class inherited by robot handlers in Simulator.java. Provides a
 * common type so that both type of handlers may be treated the same by the
 * system.
 *
 * @author geirhei
 */
abstract class RobotHandler extends Thread {
    private boolean paused = false;
    
    /**
     * Default constructor
     */
    RobotHandler() {
    }
    
    void pause() {
        paused = true;
    }

    void unpause() {
        paused = false;
    }
    
    boolean isPaused() {
        return paused;
    }
    
    @Override
    abstract public void run();
}
