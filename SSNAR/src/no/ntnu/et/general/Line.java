/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.general;

import no.ntnu.et.map.MapLocation;
import no.ntnu.et.simulator.Feature;

/**
 * This class is used to represent lines by specifying the start point of the
 * line, its direction and its length.'
 * 
 * @author Eirik Thon
 */
public class Line {
    
    private double[] start;
    
    private double[] direction;
    
    private double length;

    /**
     * Creates a new Line object
     * @param start
     * @param direction
     * @param length 
     */
    public Line(double[] start, double[] direction, double length) {
        this.start = start;
        this.direction = direction;
        this.length = length;
    }

    /**
     * Returns the start position of the line
     * @return double[] start position 
     */
    public double[] getStart() {
        return start;
    }

    /**
     * Returns the two values specifying the direction of the line
     * @return double[]
     */
    public double[] getDirection() {
        return direction;
    }

    /**
     * Returns the length of the line
     * @return double
     */
    public double getLength() {
        return length;
    }
    
    /**
     * Creates a new Line object similar to the input Feature object. The Line
     * object starts in the start position of the feature and ends in its end 
     * position
     * @param feature Feature
     * @return 
     */
    public static Line convertFeatureToLine(Feature feature){
        double[] start = new double[2];
        start[0] = feature.getStartPosition().getXValue();
        start[1] = feature.getStartPosition().getYValue();
        double length = feature.getLength();
        double xDiff = feature.getEndPosition().getXValue()-feature.getStartPosition().getXValue();
        double yDiff = feature.getEndPosition().getYValue()-feature.getStartPosition().getYValue();
        double[] vector = {xDiff/length, yDiff/length};
        return new Line(start, vector, length);
    }
    
    /**
     * Creates a new Line object between the two Position given in the input
     * parameters
     * @param startPos Position
     * @param endPos Position
     * @return Line
     */
    public static Line getLineBetweenPositions(Position startPos, Position endPos){
        double length = Position.distanceBetween(startPos, endPos);
        if(length == 0){
            return null;
        }
        double[] start = new double[2];
        start[0] = startPos.getXValue();
        start[1] = startPos.getYValue();
        double xDiff = endPos.getXValue()-startPos.getXValue();
        double yDiff = endPos.getYValue()-startPos.getYValue();
        double[] vector = {xDiff/length, yDiff/length};
        return new Line(start, vector, length);
    }
    
    /**
     * Prints all the values of the Line object
     */
    public void print(){
        System.out.println("Start: " + start[0] + ", " + start[1]);
        System.out.println("Direction: " + direction[0] + ", " + direction[1]);
        System.out.println("Length: " + length);
    }
}
