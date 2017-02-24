/**
 * This code is written as part of a Master Thesis
 * the spring of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 */
package no.ntnu.ge.slam;

import static java.lang.Math.abs;
import java.util.ArrayList;
import no.ntnu.et.map.MapLocation;

/**
 * Class for representing a 2D array representing the moving window map used
 * for navigation in a SlamRobot. Write and read operations to the array are
 * protected by synchronization on mapLock. It is therefor thread safe.
 * 
 * @author geirhei
 */
public class LocalMap {
    private int[][] map;
    private final int height;
    private final int width;
    private final Object mapLock = new Object();
    private int globalStartRow = 0;
    private int globalStartColumn = 0;
    private final int cellSize = 2;
    private boolean orientationChanged = false;
    private final int centerRow = 24;
    private final int centerColumn = centerRow;
    private final MapLocation centerLocation = new MapLocation(centerRow, centerColumn);
    
    /**
     * Constructor
     * 
     * @param height
     * @param width
     * @param orientation
     */
    public LocalMap(int height, int width) {
        this.height = height;
        this.width = width;
        map = new int[this.height][this.width];
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                map[i][j] = 2; // unexplored
            }
        }
    }
    
    MapLocation getCenterLocation() {
        return centerLocation;
    }
    
    int getCenterRow() {
        return centerRow;
    }
    
    int getCenterColumn() {
        return centerColumn;
    }
    
    boolean getOrientationChanged() {
        return orientationChanged;
    }
    
    void setOrientationChanged(boolean changed) {
        orientationChanged = changed;
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
    
    void rotateWindow(int angle, MapLocation center) {
        int centerRow = center.getRow();
        int topRow = centerRow + width/2;
        int bottomRow = centerRow - width/2;
        if (angle == 90 && angle == -90) {
            // Copy to top of array
            for (int i = 0; i < height/2; i++) {
                for (int j = 0; j < width; j++) {
                    map[height-1-i][j] = map[topRow-i][j];
                }
            }
            if (angle == 90) {
                for (int k = 0; k < height/2; k++) {
                    for (int l = 0; l < width; l++) {
                        map[k][l] = map[width-1-l][k];
                    }
                }
            } else if (angle == -90) {
                
            }
            
        } else if (angle == 180) {
            
        } else {
            //invalid angle
        }
        
        // Set top of window to unexplored
        for (int i = 0; i < height/2; i++) {
            for (int j = 0; j < width; j++) {
                map[height/2+i][j] = 2; // unexplored
            }
        }
        
    }
    
    void testFillWindow() {
        for (int i = 0; i < height; i ++) {
            for (int j = 0; j < width; j++) {
                map[i][j] = 2;
            }
        }
    }
    
    /**
     * Compares two map locations, determines if the data in the map needs to
     * be shifted because of movement, and calls the correct shift operation
     * on the array.
     * 
     * @param currentLoc
     * @param newLoc
     * @return true if a shift occurred
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
     * 
     * @param location
     * @param measurement 
     */
    public void addMeasurement(MapLocation location, boolean measurement) {
        int row = location.getRow();
        int col = location.getColumn();
        // System.out.println("Row: " + row + ", Col: " + col);
        try {
            synchronized (mapLock) {
                if (measurement) {
                    map[row][col] = 1; //occupied
                    //addRestrictingCells(location);
                    //System.out.println("Row: " + row + ", Col: " + col);
                } else {
                    if (map[row][col] == 1) {
                        //removeRestrictingCells(location);
                    }
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
    
    void addRobotWindowLocation(MapLocation robotWindowLocation) {
        int row = robotWindowLocation.getRow();
        int column = robotWindowLocation.getColumn();
        map[row][column] = 9;
    }
    
    /**
     * Based on createCircle by Thon (2016)
     * @param location 
     */
    private void addRestrictingCells(MapLocation location) {
        int radius = 15/cellSize;
        int row = location.getRow();
        int column = location.getColumn();
        int top = row + radius;
        if (top > height-1) {
            top = height-1;
        }
        int bottom = row - radius;
        if (bottom < 0) {
            bottom = 0;
        }
        int right = column + radius;
        if (right > width-1) {
            right = width-1;
        }
        int left = column - radius;
        if (left < 0) {
            left = 0;
        }
        for (int i = bottom; i <= top; i++) {
            for (int j = left; j <= right; j++) {
                if ((i-row)*(i-row)+(j-column)*(j-column) <= radius*radius) {
                    if (map[i][j] != 1 && map[i][j] != 2) {
                        map[i][j] = 3; //restricted
                    }
                }
            }
        }
    }
    
    private void removeRestrictingCells(MapLocation location) {
        int radius = 15/cellSize;
        int row = location.getRow();
        int column = location.getColumn();
        int top = row + radius;
        if (top > height-1) {
            top = height-1;
        }
        int bottom = row - radius;
        if (bottom < 0) {
            bottom = 0;
        }
        int right = column + radius;
        if (right > width-1) {
            right = width-1;
        }
        int left = column - radius;
        if (left < 0) {
            left = 0;
        }
        for (int i = bottom; i <= top; i++) {
            for (int j = left; j <= right; j++) {
                if ((i-row)*(i-row)+(j-column)*(j-column) <= radius*radius) {
                    if (map[i][j] != 1 && map[i][j] != 2) {
                        map[i][j] = 0; //free
                    }
                }
            }
        }
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
                } else if (map[i][j] == 9) {
                    System.out.print("@");
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
