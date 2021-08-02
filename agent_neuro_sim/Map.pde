public class Map {
    
    public final int NOTHING = 0;
    public final int WALL = 255;
    public final int GOAL = 1;
    public final int ROBOT = 254;
    
    public final int UP = 1;
    public final int RIGHT = 2;
    public final int DOWN = 3;
    public final int LEFT = 4;
    
    public final int RESET_MIN = 250;
    
    private  final double DEFAULT_DIM_X = 33.0;
    private  final double DEFAULT_DIM_Y = 33.0;
    
    private int sizeX;
    private int sizeY;
    private double dimX;
    private double dimY;
    private int[][] map;
    private int rX, rY;
    
    public Map(int sizeX, int sizeY) {
        this(sizeX, sizeY, 33, 33);
    }
    
    public Map(int sizeX, int sizeY, double dimX, double dimY) {
        this.sizeX = sizeX;
        this.sizeY = sizeY;
        this.dimX = DEFAULT_DIM_X;
        this.dimY = DEFAULT_DIM_Y;
        
        this.map = buildMap(sizeX, sizeY);
    }
    
    public int[][] getMap() {
        return map;
    }
    
    public float[][] getFMap(){
      float[][] ret = zeros(sizeY, sizeX);
      for(int j=0; j<sizeY; j++)
      {
        for(int i=0; i<sizeX; i++)
        {
          ret[j][i] = (float) map[j][i];
        }
      }
      
      return ret;
      
    }
    
    public int getSizeX() {
        return sizeX;
    }
    
    public int getSizeY() {
        return sizeY;
    }
    
    public void placeValue(int x, int y, int value) {
        if (! coordinateInRange(x, y)) {
            return;
        }
        this.map[x][y] = value;
        if(value == ROBOT) { rX = x; rY = y; }
    }
    
    public void updateRobotPos(int dir){
      placeValue(rX, rY, NOTHING);
      // note: x is rows, y is cols here
      if(dir == DOWN) placeValue(rX+1, rY, ROBOT);
      else if (dir == UP) placeValue(rX-1, rY, ROBOT);
      else if (dir == RIGHT) placeValue(rX, rY+1, ROBOT);
      else if (dir == LEFT) placeValue(rX, rY-1, ROBOT);
    }
    
    public MinValueDirection minSurroundingNode(int x, int y) {
        MinValueDirection mvd = new MinValueDirection(RESET_MIN, NOTHING);;
        
        if (! coordinateInRange(x, y)) {
            return mvd;
        }
        
        if (nodeLessThanMinimum(x + 1, y, mvd.getNodeValue())) {
            mvd.setNodeValue(map[x + 1][y]);
            mvd.setDirection(DOWN);
        }
                
        if (nodeLessThanMinimum(x - 1, y, mvd.getNodeValue())) {
            mvd.setNodeValue(map[x - 1][y]);
            mvd.setDirection(UP);
        }
                
        if (nodeLessThanMinimum(x, y + 1, mvd.getNodeValue())) {
            mvd.setNodeValue(map[x][y + 1]);
            mvd.setDirection(RIGHT);
        }
                
        if (nodeLessThanMinimum(x, y - 1, mvd.getNodeValue())) {
            mvd.setNodeValue(map[x][y - 1]);
            mvd.setDirection(LEFT);
        }
                
        return mvd;
    }
    
    public void clear() {
        for (int x=0; x<sizeX; x++) {
            for (int y=0; y<sizeY; y++) {
                if (map[x][y] != ROBOT && map[x][y] != GOAL) {
                    map[x][y] = NOTHING;
                }
            }
        }
    }
    
    public void unpropagate() {
        for (int x=0; x<sizeX; x++) {
            for (int y=0; y<sizeY; y++) {
                if (map[x][y] != ROBOT && map[x][y] != GOAL && map[x][y] != WALL) {
                    map[x][y] = NOTHING;
                }
            }
        }
    }
    
    public int propagateWavefront() {
        unpropagate();
        
        for (int i=0; i<50; i++) {
            for (int x=0; x<sizeX; x++) {
                for (int y=0; y<sizeY; y++) {
                    if ((map[x][y] == WALL || map[x][y] == GOAL) && map[x][y] != ROBOT) {
                        continue;
                    }
                    
                    MinValueDirection mvd = minSurroundingNode(x, y);
                    if (mvd.getNodeValue() < RESET_MIN && map[x][y] == ROBOT) {
                        return mvd.getDirection();
                    } else if (mvd.getNodeValue() != RESET_MIN) {
                        map[x][y] = mvd.getNodeValue() + 1;
                    }
                }
            }
        }
        return NOTHING;
    }
    
    public Coordinate gridLocationFromCenterRadius(int x, int y, double angle, double radius) {
        // Determine the physical location of the X, Y location
        double physX = x * dimX + (dimX / 2.0);
        double physY = y * dimY + (dimY / 2.0);
        double radAngle = angle * Math.PI / 180.0;
        double targetX = physX + radius * Math.cos(radAngle);
        double targetY = physY + radius * Math.sin(radAngle);

        // Hand back grid tuple
        return new Coordinate((int)Math.round((targetX - dimX / 2.0) / dimX),
                              (int)Math.round((targetY - dimY / 2.0) / dimY));
//    [ ((target_x - dim_x / 2.0) / dim_x).round,
//      ((target_y - dim_y / 2.0) / dim_y).round ]
  // end
    }

    
    private boolean coordinateInRange(int x, int y) {
        return x >= 0 && x < this.sizeX && y >= 0 && y < this.sizeY;
    }
    
    private boolean nodeLessThanMinimum(int x, int y, int minimum) {
        boolean lessThanMinimum = false;
        if (coordinateInRange(x, y) && this.map[x][y] != NOTHING && map[x][y] < minimum) {
            lessThanMinimum = true;
        }
        return lessThanMinimum;
    }
    
    private int[][] buildMap(int sizeX, int sizeY) {
        int[][] map = new int[sizeX][sizeY];
        
        // Initialize the cells to NOTHING
        for (int x=0; x<sizeX; x++) {
            for (int y=0; y<sizeY; y++) {
                map[x][y] = NOTHING;
            }
        }
        
        return map;
    }

}


class Coordinate {
    
    private int x;
    private int y;
    
    public Coordinate(int x, int y) {
        this.x = x;
        this.y = y;
    }
    
    public void setX(int x) {
        this.x = x;
    }
    
    public int getX() {
        return x;
    }

    public void setY(int y) {
        this.y = y;
    }
    
    public int getY() {
        return y;
    }
    
    public boolean onTheGrid(int maxX, int maxY) {
        return x >= 0 && x < maxX && y >= 0 && y < maxY;
    }

}


class MinValueDirection {
    private int nodeValue;
    private int direction;
    private int nullDirection;
    
    public MinValueDirection(int value, int direction) {
        this.nodeValue = value;
        this.direction = direction;
        this.nullDirection = direction;
    }
    
    public boolean directionSet() {
        return this.direction != this.nullDirection;
    }
    
    public void setNodeValue(int value) {
        this.nodeValue = value;
    }
    
    public int getNodeValue() {
        return this.nodeValue;
    }
    
    public void setDirection(int direction) {
        this.direction = direction;
    }
    
    public int getDirection() {
        return this.direction;
    }

}
