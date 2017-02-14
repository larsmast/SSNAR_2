/**
 * This code is written as part of a Master Thesis
 * the spring of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 */
package no.ntnu.ge.slam;

import static java.lang.Math.abs;
import no.ntnu.et.map.MapLocation;

/**
 * Class for representing a 2D array representing the moving window map used
 * for navigation in a SlamRobot. Write and read operations to the array are
 * protected by synchronization on mapLock. It is therefor thread safe.
 * 
 * @author geirhei
 */
public class WindowMap {
    private int[][] map;
    private final int height;
    private final int width;
    private final Object mapLock = new Object();
    private int globalStartRow = 0;
    private int globalStartColumn = 0;
    /*
    private MapLocation[] frontierLocations;
    private MapLocation[] occupiedLocations;
    */
    
    /**
     * Constructor
     * 
     * @param height
     * @param width 
     */
    public WindowMap(int height, int width) {
        this.height = height;
        this.width = width;
        map = new int[this.height][this.width];
        init();
        /*
        frontierLocations = new MapLocation[500];
        occupiedLocations = new MapLocation[500];
        */
    }
    
    private void init() {
        synchronized (mapLock) {
            for (int i = 0; i < height; i++) {
                for (int j = 0; j < width; j++) {
                    map[i][j] = 2; // unexplored
                }
            }
        }
    }
    
    int getGlobalStartRow() {
        return globalStartRow;
    }
    
    int getGlobalStartColumn() {
        return globalStartColumn;
    }
    
    void setGlobalStartRow(int row) {
        globalStartRow = row;
    }
    
    void setGlobalStartColumn(int column) {
        globalStartColumn = column;
    }
    
    int[][] getWindow() {
        return map;
    }
    
    /**
     * Getter for height.
     * 
     * @return height of the map
     */
    int getHeight() {
        return this.height;
    }
    
    /**
     * Getter for width.
     * 
     * @return width of the map
     */
    int getWidth() {
        return this.width;
    }
    
    /**
     * Compares two map locations, determines if the data in the map needs to
     * be shifted because of movement, and calls the correct shift operation
     * on the array.
     * 
     * @param currentLoc
     * @param newLoc
     * @return true if a shift occured
     */
    public boolean shift(MapLocation currentLoc, MapLocation newLoc) {
        int dx = newLoc.getColumn() - currentLoc.getColumn();
        int dy = newLoc.getRow() - currentLoc.getRow();
        if (dx == 0 && dy == 0) {
            return false;
        } else {
            if (dx > 0) {
                for (int i = 0; i < dx; i++) {
                    shiftLeft();
                }
                //System.out.println("Shift left: " + dx);
            } else if (dx < 0) {
                for (int j = 0; j < abs(dx); j++) {
                    shiftRight();
                }
                //System.out.println("Shift right: " + dx);
            }
            if (dy > 0) {
                for (int k = 0; k < dy; k++) {
                    shiftDown();
                }
                //System.out.println("Shift down: " + dy);
            } else if (dy < 0) {
                for (int l = 0; l < abs(dy); l++) {
                    shiftUp();
                }
                //System.out.println("Shift up: " + dy);
            }
            return true;
        }
    }
    
    /**
     * Shifts the content of the map to the right.
     */
    private void shiftRight() {
        synchronized (mapLock) {
            for (int i = 0; i < height; i++) {
                for (int j = width-1; j > 0; j--) {
                    map[i][j] = map[i][j-1];
                }
            }
            for (int k = 0; k < height; k++) {
                map[k][0] = 2;
            }
        }
    }
    
    private void shiftLeft() {
        synchronized (mapLock) {
            for (int i = 0; i < height; i++) {
                for (int j = 0; j < width - 1; j++) {
                    map[i][j] = map[i][j+1];
                }
            }
            for (int k = 0; k < height; k++) {
                map[k][width-1] = 2;
            }
        }
    }
    
    private void shiftUp() {
        synchronized (mapLock) {
            for (int i = height-1; i > 0; i--) {
                for (int j = 0; j < width; j++) {
                    map[i][j] = map[i-1][j];
                }
            }
            for (int k = 0; k < width; k++) {
                map[0][k] = 2;
            }
        }
    }
    
    private void shiftDown() {
        synchronized (mapLock) {
            for (int i = 0; i < height-1; i++) {
                for (int j = 0; j < width; j++) {
                    map[i][j] = map[i+1][j];
                }
            }
            for (int k = 0; k < width; k++) {
                map[height-1][k] = 2;
            }
        }
    }
    
    /**
     * Updates the map.
     * (This method also updates the restricted and weakly
     * restricted area of the map if the occupied status of a cell changes.)
     * 
     * @param location
     * @param measurement 
     */
    public void addMeasurement(MapLocation location, boolean measurement) {
        int row = location.getRow();
        int col = location.getColumn();
        //System.out.println("Row: " + row + ", Col: " + col);
        try {
            synchronized (mapLock) {
                if (measurement) {
                    map[row][col] = 1; //occupied
                } else {
                    map[row][col] = 0; // free
                } // (unexplored = 2)
            }
        } catch (ArrayIndexOutOfBoundsException e) {
            System.err.println("ArrayIndexOutOfBoundsException in addMeasurement: " + e.getMessage());
            System.err.println("Row: " + row + ", Col: " + col);
        }
        

        // If the cell changes from occupied to free or vice versa, the restricted
        // status of nearby cells are updated here:
        /*
        if(measuredCell.stateChanged()){
            ArrayList<MapLocation> restricted = createCircle(location, 15);
            ArrayList<MapLocation> weaklyRestricted = createCircle(location, 25);
            for(MapLocation location2: restricted){
                if(measuredCell.isOccupied()){
                    map.get(location2).addRestrictingCell(measuredCell);
                }
                else {
                    map.get(location2).removeRestrictingCell(measuredCell);
                }
            }
            for(MapLocation location2: weaklyRestricted){
                if(measuredCell.isOccupied()){
                    map.get(location2).addWeaklyRestrictingCell(measuredCell);
                }
                else {
                    map.get(location2).removeWeaklyRestrictingCell(measuredCell);
                }
            }
        }
        */
    }
    
    /**
     * Print method for debugging.
     */
    public void print() {
        for (int i = height-1; i >= 0; i--) {
            for (int j = 0; j < width; j++) {
                if (map[i][j] == 1) {
                    System.out.print('X');
                } else if (map[i][j] == 2) {
                    System.out.print('#');
                } else {
                    System.out.print(map[i][j]);
                }
                System.out.print(" ");
            }
            System.out.println();
        }
        System.out.println("------------------------------------------");
    }
}
