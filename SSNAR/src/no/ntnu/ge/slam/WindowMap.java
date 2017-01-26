/**
 * This code is written as part of a Master Thesis
 * the spring of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 */
package no.ntnu.ge.slam;

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
        initWindowMap();
    }
    
    private void initWindowMap() {
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                map[i][j] = 2; // unexplored
            }
        }
    }
}
