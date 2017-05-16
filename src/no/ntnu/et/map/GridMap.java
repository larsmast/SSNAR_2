/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.map;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentHashMap;
import no.ntnu.et.general.Position;

/**
 * This class represents a grid map. A hash table of MapLocations and Cells are
 * used to represent the actual map. The map expand in all directions. Expansion
 * downwards and to the left will lead to negative rows and columns. The map
 * contains variables to specify the indexes of the columns and rows at its
 * boundaries.
 * 
 * ABOUT THE GRID: The cells of the grid are square areas determined by the
 * cell size. The grid is implemented so that it is connected to a coordinate
 * system. It is easiest to explain how this works with an example:
 * 
 * If the cell size is set to 5 cm, all positions in the coordinate system
 * with x-values from [0, 5) are mapped into the first column. Positions with
 * x-values [5,10) are mapped to the second column and so on. The same principle
 * applies to y-values and rows. For example the position (0,0) is has the
 * indexes (row=0,col=0) in the map The position (4,3) also has the indexes
 * (row=0,col=0). The position (5,1) maps to (row=0,col=1).
 * 
 * This principle also continues over to the negative side of the coordinate
 * system. With 5 cm cell size: (-1,3) maps to (row=0, col=-1), and (-5,3) maps
 * to (row=0, col=-2).
 * 
 * This may be confusing, but it (hopefully) makes sense after a while 
 * 
 * @author Eirik Thon
 */

public class GridMap{
    private ConcurrentHashMap<MapLocation, Cell> map;
    private int cellSize;
    private int topRow;
    private int bottomRow;
    private int rightColumn;
    private int leftColumn;
    
    /**
     * Constructor for the GridMap class
     * @param cellSize Specifies the size of cells in cm. Cells are quadratic
     * so no extra dimension is needed
     * @param width Specifies the initial width of the map in cm.
     * @param height Specifies the initial height of the map in cm
     */
    public GridMap(int cellSize, int width, int height) {
        if (cellSize <= 0 || width <= 0 || height <= 0 || width % cellSize != 0 || height % cellSize != 0){
            System.out.println("Error. All parameters must be positive and width and height must be a positive multiple of cellSize");
        }else{
            this.cellSize = cellSize;
            map = new ConcurrentHashMap<MapLocation, Cell>();
            topRow = height/cellSize-1;
            bottomRow = 0;
            rightColumn = width/cellSize-1;
            leftColumn = 0;
            for (int i = bottomRow; i <= topRow; i++) {
                for (int j = leftColumn; j <= rightColumn; j++) {
                    map.put(new MapLocation(i, j), new Cell());
                }
            }
        }
    }
    
    /**
     * Returns the total number of cells in the map (NOT USED)
     * @return 
     */
    public int getNumberOfCells(){
        return map.size();
    }

    /**
    * Returns the index top row of the map
    * @return 
    */
    public int getTopRow() {
        return topRow;
    }

    /**
    * Returns the index bottom row of the map
    * @return 
    */
    public int getBottomRow() {
        return bottomRow;
    }

    /**
    * Returns the index of the rightmost column of the map
    * @return 
    */
    public int getRightColumn() {
        return rightColumn;
    }

    /**
    * Returns the index of the leftmost column of the map
    * @return 
    */
    public int getLeftColumn() {
        return leftColumn;
    }
    
    /**
     * Returns the size of the cells
     * @return 
     */
    public int getCellSize(){
        return cellSize;
    }
    
    /**
     * Returns the total number of rows
     * @return 
     */
    public int getNumberOfRows(){
        return topRow-bottomRow+1;
    }
    
    /**
     * Returns the total number of columns
     * @return 
     */
    public int getNumberOfColumns(){
        return rightColumn-leftColumn+1;
    }
    
    /**
     * Returns the width of the map in cm (NOT USED)
     * @return 
     */
    public int getMapWidth(){
        return getNumberOfColumns()*cellSize;
    }
    
    /**
     * Returns the height of the map in cm (NOT USED)
     * @return 
     */
    public int getMapHeight(){
        return getNumberOfRows()*cellSize;
    }
    
    /**
     * Returns the cell at the specified location
     * @param location
     * @return 
     */
    public Cell findCell(MapLocation location){
        //
        return map.get(location);
    }

    /**
     * Returns the map
     * @return 
     */
    public ConcurrentHashMap<MapLocation, Cell> getMap(){
        return map;
    }
    
    /**
     * Updates the map. This method also updates the restricted and weakly
     * restricted area of the map if the occupied status of a cell changes.
     * @param location
     * @param measurement 
     */
    public void addMeasurement(MapLocation location, boolean measurement) {
        Cell measuredCell = map.get(location);
        measuredCell.update(measurement);

        // If the cell changes from occupied to free or vice versa, the restricted
        // status of nearby cells are updated here:
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
    }
    
    /**
     * Finds and returns the MapLocation of all cells that are free, not
     * restricted and has an unobserved neighbor
     * @return 
     */
    public ArrayList<MapLocation> getFrontierLocations() {
        ArrayList<MapLocation> frontierLocations = new ArrayList<MapLocation>();
        for(int i = bottomRow; i <= topRow; i++) {
            for(int j = leftColumn; j <= rightColumn; j++) {
                MapLocation location = new MapLocation(i, j);
                if(map.get(location).isWeaklyTargetable()) {
                    ArrayList<MapLocation> neighbors = findDirectNeighborCells(location);
                    for(MapLocation neighbor: neighbors){
                        if(!findCell(neighbor).isPreviouslyObserved()){
                            //cell.setFrontier();
                            frontierLocations.add(location);
                            break;
                        }
                    }
                }
            }
        }
        return frontierLocations;
    }
    
    /**
     * Returns the number of unexplored cells around the specified location
     * within the specified radius
     * @param location
     * @param radius
     * @return 
     */
    public int countUnknownCellsAroundLocation(MapLocation location, int radius){
        ArrayList<MapLocation> circle = createCircle(location, radius);
        int counter = 0;
        for(MapLocation location2: circle){
            if(!map.get(location2).isPreviouslyObserved()){
                counter++;
            }
        }
        return counter;
    }
    
    /**
     * See findLocationInMap(Position position).
     * @param position
     * @return 
     */
    public MapLocation findLocationInMap(int[] position) {
        return findLocationInMap(new Position(position[0], position[1]));
    }
    
    /**
     * Returns the MapLocation that corresponds to the specified position.
     * @param position
     * @return 
     */
    public MapLocation findLocationInMap(Position position) {
        int row = 0;
        if(position.getYValue() >= 0){
            row = (int)(position.getYValue()/cellSize);
        }else{
            if(position.getYValue()%cellSize == 0){
                row = (int)(position.getYValue()/cellSize);
            }else{
                row = (int)(position.getYValue()/cellSize)-1;
            }
        }
        int column = 0;
        if(position.getXValue() >= 0){
            column = (int)(position.getXValue()/cellSize);
        }else{
            if(position.getXValue()%cellSize == 0){
                column = (int)(position.getXValue()/cellSize);
            }else{
                column = (int)(position.getXValue()/cellSize)-1;
            }
        }
        return new MapLocation(row, column);
    }
    
    /**
     * Returns an array of MapLocations that creates a circle around the 
     * specified location within the specified radius. If any part of the circle
     * exceeds the boundaries of the map, those MapLocations are not included in
     * the array
     * @param location
     * @param radius
     * @return 
     */
    ArrayList<MapLocation> createCircle(MapLocation location, int radius){
        ArrayList<MapLocation> circle = new ArrayList<MapLocation>();
        radius = radius/cellSize;
        int row = location.getRow();
        int column = location.getColumn();
        int top = row + radius;
        if (top > topRow) {
            top = topRow;
        }
        int bottom = row - radius;
        if (bottom < bottomRow) {
            bottom = bottomRow;
        }
        int right = column + radius;
        if (right > rightColumn) {
            right = rightColumn;
        }
        int left = column - radius;
        if (left < leftColumn) {
            left = leftColumn;
        }
        for(int i = bottom; i <= top; i++) {
            for(int j = left; j <= right; j++) {
                MapLocation otherLocation = new MapLocation(i, j);
                if((i-row)*(i-row)+(j-column)*(j-column) <= radius*radius){
                    circle.add(otherLocation);
                }
            }
        }
        return circle;
    }
    
    /**
     * Adds the necessary rows and/or columns to the map so that the specified
     * position is included in the map.
     * @param position 
     */
    public void resize(Position position) {
        MapLocation location = findLocationInMap(position);
        int row = location.getRow();
        int column = location.getColumn();
        if (row > topRow) {
            addRowsTop(row-topRow);
        }
        else if (row < bottomRow) {
            addRowsBottom(Math.abs(row)-Math.abs(bottomRow));
        }
        if (column > rightColumn) {
            addColumnsRight(column-rightColumn);
        }
        else if (column < leftColumn) {
            addColumnsLeft(Math.abs(column)-Math.abs(leftColumn));
        }
    }

    /**
     * Creates the specified amount of rows at the bottom of the map
     * @param numberOfRows 
     */
    private void addRowsBottom(int numberOfRows) {
        for (int i = bottomRow-numberOfRows; i < bottomRow; i++) {
            for (int j = leftColumn; j <= rightColumn; j++) {
                MapLocation location = new MapLocation(i,j);
                map.put(location, new Cell());
                ArrayList<MapLocation> restrictedCircle = createCircle(location, 15);
                for(MapLocation otherLoc: restrictedCircle){
                    if(map.get(otherLoc).isOccupied()){
                        map.get(location).addRestrictingCell(map.get(otherLoc));
                        break;
                    }
                }
                ArrayList<MapLocation> weaklyRestrictedCircle = createCircle(location, 25);
                for(MapLocation otherLoc: weaklyRestrictedCircle){
                    if(map.get(otherLoc).isOccupied()){
                        map.get(location).addWeaklyRestrictingCell(map.get(otherLoc));
                        break;
                    }
                }
            }
        }
        bottomRow -= numberOfRows;
    }
    
     /**
     * Creates the specified amount of rows at the top of the map
     * @param numberOfRows 
     */
    private void addRowsTop(int numberOfRows) {
        for (int i = topRow+1; i <= topRow+numberOfRows; i++) {
            for (int j = leftColumn; j <= rightColumn; j++) {
                MapLocation location = new MapLocation(i,j);
                map.put(location, new Cell());
                ArrayList<MapLocation> restrictedCircle = createCircle(location, 15);
                for(MapLocation otherLoc: restrictedCircle){
                    if(map.get(otherLoc).isOccupied()){
                        map.get(location).addRestrictingCell(map.get(otherLoc));
                        break;
                    }
                }
                ArrayList<MapLocation> weaklyRestrictedCircle = createCircle(location, 25);
                for(MapLocation otherLoc: weaklyRestrictedCircle){
                    if(map.get(otherLoc).isOccupied()){
                        map.get(location).addWeaklyRestrictingCell(map.get(otherLoc));
                        break;
                    }
                }
            }
        }
        topRow += numberOfRows;
    }
    
     /**
     * Creates the specified amount of columns at the left side of the map
     * @param numberOfRows 
     */
    private void addColumnsLeft(int numberOfColumns) {
        for (int i = bottomRow; i <= topRow; i++) {
            for (int j = leftColumn-numberOfColumns; j < leftColumn; j++) {
                MapLocation location = new MapLocation(i,j);
                map.put(location, new Cell());
                ArrayList<MapLocation> restrictedCircle = createCircle(location, 15);
                for(MapLocation otherLoc: restrictedCircle){
                    if(map.get(otherLoc).isOccupied()){
                        map.get(location).addRestrictingCell(map.get(otherLoc));
                        break;
                    }
                }
                ArrayList<MapLocation> weaklyRestrictedCircle = createCircle(location, 25);
                for(MapLocation otherLoc: weaklyRestrictedCircle){
                    if(map.get(otherLoc).isOccupied()){
                        map.get(location).addWeaklyRestrictingCell(map.get(otherLoc));
                        break;
                    }
                }
            }
        }
        leftColumn -= numberOfColumns;
    }
    
    /**
     * Creates the specified amount of columns at the right side of the map
     * @param numberOfRows 
     */
    private void addColumnsRight(int numberOfColumns) {
        for (int i = bottomRow; i <= topRow; i++) {
            for (int j = rightColumn+1; j <= rightColumn+numberOfColumns; j++) {
                MapLocation location = new MapLocation(i,j);
                map.put(location, new Cell());
                ArrayList<MapLocation> restrictedCircle = createCircle(location, 15);
                for(MapLocation otherLoc: restrictedCircle){
                    if(map.get(otherLoc).isOccupied()){
                        map.get(location).addRestrictingCell(map.get(otherLoc));
                        break;
                    }
                }
                ArrayList<MapLocation> weaklyRestrictedCircle = createCircle(location, 25);
                for(MapLocation otherLoc: weaklyRestrictedCircle){
                    if(map.get(otherLoc).isOccupied()){
                        map.get(location).addWeaklyRestrictingCell(map.get(otherLoc));
                        break;
                    }
                }
            }
        }
        rightColumn += numberOfColumns;
    }
    
    /**
     * Creates and returns new MapLocations that are directly next to the
     * specified location. Does not return MapLocations that are outside the
     * map's boundaries
     * @param location
     * @return 
     */
    public ArrayList<MapLocation> findDirectNeighborCells(MapLocation location){
        ArrayList<MapLocation> neighbors = new ArrayList<MapLocation>();
        int row = location.getRow();
        int column = location.getColumn();
        if(row + 1 <= topRow){ // Above
            neighbors.add(new MapLocation(row+1, column));
        }
        if(column + 1 <= rightColumn){ // Right
            neighbors.add(new MapLocation(row, column+1));
        }
        if(row - 1 >= bottomRow){ // Below
            neighbors.add(new MapLocation(row-1, column));
        }
        if(column - 1 >= leftColumn){ // Left
            neighbors.add(new MapLocation(row, column-1));
        }
        return neighbors;
    }
    
    
    /**
     * Creates and returns new MapLocations that are diagonally connected to the
     * specified location. Does not return MapLocations that are outside the
     * map's boundaries
     * @param location
     * @return 
     */
    public ArrayList<MapLocation> findDiagonalNeighborCells(MapLocation location){
        ArrayList<MapLocation> neighbors = new ArrayList<MapLocation>();
        int row = location.getRow();
        int column = location.getColumn();
        if(row + 1 <= topRow && column + 1 <= rightColumn){ // Above right
            neighbors.add(new MapLocation(row+1, column+1));
        }
        if(row + 1 <= topRow && column - 1 >= leftColumn){ // Above left 
            neighbors.add(new MapLocation(row+1, column-1));
        }
        if(row - 1 >= bottomRow && column + 1 <= rightColumn){ // Below right
            neighbors.add(new MapLocation(row-1, column+1));
        }
        if(row - 1 >= bottomRow && column - 1 >= leftColumn){ // Below left
            neighbors.add(new MapLocation(row-1, column-1));
        }
        return neighbors;
    }
    
    /**
     * Returns the position position of the center of the specified MapLocation
     * @param location
     * @return 
     */
    public Position mapLocation2Position(MapLocation location){
        return new Position((double)(location.getColumn()*cellSize)+(double)cellSize/2, (double)(location.getRow()*cellSize)+(double)cellSize/2);
    }
    
    /**
     * Fills in small unexplored areas of the map with free space
     */
    public void cleanUp(){
        for(int i = bottomRow; i <= topRow; i++) {
            for(int j = leftColumn; j <= rightColumn; j++) {
                MapLocation location = new MapLocation(i, j);
                if(!map.get(location).isPreviouslyObserved()){
                    ArrayList<MapLocation> neighbors = findDirectNeighborCells(location);
                    for(MapLocation neighbor: neighbors){
                        if(map.get(neighbor).isPreviouslyObserved()){
                            if(countUnknownCellsAroundLocation(location, 5)*cellSize*cellSize < 23){
                                map.get(location).update(false);
                            }
                            break;
                        }
                    }
                }
            }
        }
    }
}
