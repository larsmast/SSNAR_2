/**
 * This code is written as part of a Master Thesis
 * the spring of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 */
package no.ntnu.ge.slam;

import no.ntnu.et.map.MapLocation;

/**
 *
 * @author geirhei
 */
public class WindowMap {
    private int[][] map;
    private final int height;
    private final int width;
    private final Object mapLock = new Object();
    
    public WindowMap(int height, int width) {
        this.height = height;
        this.width = width;
        map = new int[this.height][this.width];
        init();
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
    
    int getHeight() {
        return this.height;
    }
    
    int getWidth() {
        return this.width;
    }
    
    public boolean shift(MapLocation currentLoc, MapLocation newLoc) {
        int dx = newLoc.getColumn() - currentLoc.getColumn();
        int dy = newLoc.getRow() - currentLoc.getRow();
        if (dx == 0 && dy == 0) {
            return false;
        } else {
            if (dx > 0) {
                shiftLeft();
                //System.out.println("Shift left: " + dx);
            } else if (dx < 0) {
                shiftRight();
                //System.out.println("Shift right: " + dx);
            }
            if (dy > 0) {
                shiftDown();
                //System.out.println("Shift down: " + dy);
            } else if (dy < 0) {
                shiftUp();
                //System.out.println("Shift up: " + dy);
            }
            return true;
        }
    }
    
    private void shiftRight() {
        synchronized (mapLock) {
            for (int i = 0; i < height; i++) {
                for (int j = 0; j < width - 1; j++) {
                    map[i][j+1] = map[i][j];
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
            for (int i = 1; i < height; i++) {
                for (int j = 0; j < width; j++) {
                    map[i-1][j] = map[i][j];
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
    
    public void print() {
        for (int i = height-1; i >= 0; i--) {
            for (int j = 0; j < width; j++) {
                if (map[i][j] == 1) {
                    System.out.print('X');
                } else {
                    System.out.print(map[i][j]);
                }
            }
            System.out.println();
        }
        System.out.println("------------------------------------------");
    }
}
