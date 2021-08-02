import java.awt.Image;
import javax.swing.ImageIcon;
import java.awt.image.BufferedImage;
import java.awt.color.ColorSpace;
import java.util.Arrays;

void printArray(String s, float [] a){
  if (s!="") {print(s+": "); println();}
  for (int i=0; i<a.length; i++)
    print(Float.toString(a[i]) + "; "); 
  println();
}

void printMatrix(String s, float[][] a){
  if (s!="") {print(s+": "); println();}
  for (int i=0; i<a.length; i++)
    printArray("", a[i]); 
  println();
}

void draw_rot_line(float angle, float trans_x, float trans_y){
  float lnlen = 10;
  float wt = 2;
  draw_rot_line(angle, trans_x, trans_y, lnlen, wt);
}


void draw_rot_line(float angle, float trans_x, float trans_y, float line_len, float str_weight){
  pushMatrix();
  pushStyle();
  translate(trans_x, trans_y);
  stroke(165);
  circle(0,0,line_len*2);
  strokeWeight(str_weight);
  rotate(angle);
  line(0,0, line_len, 0);
  popStyle();
  popMatrix();
}

void draw_q_phase_line(float angle, int quant, float trans_x, float trans_y){
  float q_angle = 2*PI/(float)quant;
  float flr =  ceil(angle/q_angle); // use ceil to round negs towards zero
  float rotangle = q_angle * flr;
  // print(angle);
  // print("; ");
  // println(rotangle);
  draw_rot_line(rotangle, trans_x, trans_y);
}

/**
  float yf1, 
  float xf1, 
  float yscale, 
  float xscale, 
  color clr, 
  int labelplacement,  0 above, 1 below
  float aymax,
  String legend,
  PFont h1,
  PFont l1

  Example:
  float yscl = 47.8;
  float xscl = 134.0;
  barchart_array(
    gateIntegrator.getOutput()[0],
    0, 0,
    yscl, xscl,
    #A0EA9A,
    1,
    1.f,
    "gate",
    font_8, font_8
  );
*/
void barchart_array(float[] data,
  float yf1, 
  float xf1, 
  float yscale, 
  float xscale, 
  color clr, 
  int labelplacement,
  float aymax,
  String legend,
  PFont h1,
  PFont l1) {
    pushStyle();
  //Declare a float variabe for the max y axis value.
   float ymax=aymax;
   
   //Declare a float variable for the minimum y axis value.
   float ymin = 0;
   
   //Set the stroke color to a medium gray for the axis lines.
   stroke(175);
   
   //draw the axis lines.
   line(xf1-3,yf1+2,xf1+xscale,yf1+2);  
   line(xf1-3,yf1+2,xf1-3,yf1-yscale); 
     
   //turn off strokes for the bar charts 
   noStroke(); 
    
   //loop the chart drawing once.
   for (int c1 = 0; c1 < 1; c1++){  
   
     //Set the start x point value.
     float xfstart = xf1;  
     //Count the number of rows in the file.
     //for (int i = 0; i < lines.length; i++) {
       
       //For each line split values separated by columns into pieces.
       //String[] pieces = split(lines[i], ',');
       
       //Set the max Y axis value to be 10 greater than the max value in the pieces array.
       // ymax = 1.5* max(data);
       
       //Count the number of pieces in the array.
       float xcount = data.length;
       
       //Draw the minimum and maximum Y Axis labels. 
       textFont(h1);
       fill (100);
       textAlign(RIGHT, CENTER);
       text(int(ymax), xf1-8, yf1-yscale);
       text(int(ymin), xf1-8, yf1);
       text(legend, xf1+87, yf1+18);
       
       //Draw each column in the data series.
       for (int i2 = 0; i2 < xcount; i2++) {
         
         //Get the column value and set it has the height.
         float yheight = data[i2];
         
         //Declare the variables to hold column height as scaled to the y axis.     
         float ypct;
         float ysclhght;
         
         //calculate the scale of given the height of the chart.
         ypct = yheight / ymax;
         
         //Calculate the scale height of the column given the height of the chart.
         ysclhght = yscale * ypct;
         
         //If the column height exceeds the chart height than truncate it to the max value possible.
         if(ysclhght > yscale){
           ysclhght = yscale;
         }
         
         //Determine the width of the column placeholders on the X axis.
         float xcolumns = xscale / xcount;
         
         //Set the width of the columns to 5 pixels less than the column placeholders.
         float xwidth = xcolumns - 5;
         
         //Set the fill color of the columns.
         fill (clr);
         
         //Draw the columns to scale.
         quad(xf1, yf1, xf1, yf1-ysclhght, xf1 + xwidth, yf1-ysclhght, xf1 + xwidth, yf1);
         
         //Draw the labels.
         textFont(l1);
         textAlign(CENTER, CENTER);
         fill (100);
         
         //Decide where the labels will be placed.
         if (labelplacement < 1) {
           //Above the columns.
           text(nf(data[i2], 0, 2), xf1 + (xwidth / 2), yf1 - (ysclhght + 8));
         } else {
           //Below the columns.
           text(data[i2], xf1 + (xwidth / 2), yf1 + 8);
         }
         //increment the x point at which to draw a column.
         xf1 = xf1 + xcolumns;
       //}
    }
  }
  popStyle();
}

/**
*/
void drawGraph(float[][] units, float[][] adjmatrix){
  pushStyle();
  float inner = 10.9;
  float outer= 40.3;
  color incol = color(0,0,0,151);
  color outcol = color(222, 147);
  color txtcol = color(200);
  color excol = color(217,122,122,255);
  color inhcol= color(122,122,217,255);
  color linecol;
  for(int j=0; j<adjmatrix.length; j++){
    pushStyle();
    fill(outcol);
    circle(units[j][0], units[j][1], outer);
    fill(incol);
    circle(units[j][0], units[j][1], inner);
    popStyle();
    // text
    pushStyle();
    fill(txtcol);
    text("U" + (j+1), units[j][0], units[j][1] + outer+5);
    popStyle();
    for(int i=0; i<adjmatrix[0].length; i++){
        if(adjmatrix[j][i] != 0){
          float[] from = units[j];
          float[] to = units[i];
          pushStyle();
          strokeWeight(4);
          linecol = excol;
          if(adjmatrix[j][i] < 0)
            linecol = inhcol;
          stroke(linecol);
          arrow(from[0], from[1], to[0], to[1]);
          popStyle();
        }
    }
  }
  popStyle();
}

void arrow(float x1, float y1, float x2, float y2) {
  float pt = 7;
  line(x1, y1, x2, y2);
  pushMatrix();
  translate(x2, y2);
  float a = atan2(x1-x2, y2-y1);
  rotate(a);
  line(0, 0, -pt, -pt);
  line(0, 0, pt, -pt);
  popMatrix();
} 

void drawTopologyGrid(float x1, float y1, float dim, float[][] top){
  color exccol = color(217,122,122,255);
  color inhcol= color(122,122,217,255);
  
  float y=y1;
  float margin = 10;
  for(int j=0; j<top.length; j++){
    float x=x1;
    for(int i=0; i<top[0].length; i++){
      pushStyle();
      if(top[j][i] > 0) fill(exccol);
      else if(top[j][i] < 0) fill(inhcol);
      else fill(100);
      square(x, y, dim);
      popStyle();
      x+= dim+margin;
    }
    y+= dim+margin;
  }
}

void drawColGrid(float x1, float y1, float dim, float[][] top){
  /**
  x1 - topleft corner x
  y1 - topleft corner y
  dim - scale dim of individual square
  top - array of values: pos and neg values drawn differently
  */
  
  drawColGrid(x1, y1, dim, 10, "", top);
}

void drawColGrid(float x1, float y1, float dim, float margin, String title, float[][] top){
  /**
  x1 - topleft corner x
  y1 - topleft corner y
  dim - scale dim of individual square
  margin - between square margin
  top - array of values: pos and neg values drawn differently
  */
  color exccol = color(217,122,122,255);
  color inhcol= color(122,122,217,255);
  
  pushMatrix();
  translate(0, -10);
  text(title, 0, 0);
  popMatrix();
  
  pushStyle();
  colorMode(HSB);
  
  float y=y1;
  // float margin = 10;
  for(int j=0; j<top.length; j++){
    float x=x1;
    for(int i=0; i<top[0].length; i++){
      pushStyle();
      if(top[j][i] > 0) fill(hue(exccol), saturation(exccol), top[j][i]);
      else if(top[j][i] < 0) fill(hue(inhcol), saturation(inhcol), top[j][i]);
      else fill(0);
      square(x, y, dim);
      popStyle();
      x+= dim+margin;
    }
    y+= dim+margin;
  }
  popStyle();
}

/** Draw a time-series plot
*/
void drawTimeSeries(float[] series, float maxy, float x_margin, float thresh){
 float y_length=100;
 float x_length=100;
// float x_margin = 20;
 //float ptsz = 10;
 pushStyle();
 stroke(200);
 strokeWeight(2);
 //draw y
 line(0,0,0,2*y_length);
 line(0,y_length, x_length, y_length);
 pushStyle();
 stroke(200, 100, 0);
 float y_thr = y_length*(1 - thresh/maxy);
 line(0,  y_thr, x_length, y_thr); 
 popStyle();
 // draw data
 float x = x_margin;
 pushStyle();
 //fill(150);
 noFill();
 strokeWeight(1);
 beginShape();
 for(int i=0; i<series.length; i++){
   float y = y_length*(1 - series[i]/maxy);
   //circle(x, y,  ptsz);
   vertex(x, y);
   x+=x_margin;
 }
 endShape();
 popStyle();
 popStyle();
}



class Buffer{
  FloatList data; 
  Buffer(int sz){
    data = new FloatList(sz);
    for(int i=0; i<sz; i++)
      data.append(0);
  }
  
  float[] array(){
    return data.array();
  }
  
  float head(){
    return data.get(0);
  }
  
  float get(int ix){
    return data.get(ix);
  }
  
  void append(float val){
    data.append(val);
    data.remove(0);
  }
}

void drawSpikeStrip(Buffer[] aspikes, float threshold){
  pushStyle();
  color ptcolor = color(228, 106, 218);
  float x=0; float y=0;
  float x_i =4; float y_i=10;  
  float margin = 1;
  fill(ptcolor);
  noStroke(); //<>// //<>//
  for(int j=0; j<aspikes.length; j++){
    x=0;
    text("U"+(j+1), x-25, y+5);
    for(int i=0; i<aspikes[0].array().length; i++){
      // draw the strip //<>// //<>//
    if(aspikes[j].array()[i] > threshold){
      rect(x, y, x_i, y_i);}
    x+= x_i + margin;
    }
   y+= y_i;
  }
  popStyle();
}

int countSpikes(float[] data, float threshold){
  int retval = 0;
  for(int i=0; i< data.length; i++)
    if(data[i] >= threshold) retval++;
  return retval;
}

float[][] makeTopology(int dim, int type){
 float[][] retval = new float[dim*dim][dim*dim];
  // types
 // 1 nearest neighbor
 // 2 random
 /*
  [M, N] = size(T); //<>// //<>// //<>// //<>// //<>// //<>// //<>//
  [X, Y] = meshgrid(max(1,m-5):min(M,m+5), max(1,n-5):min(N,n+5));
  I = sub2ind(size(T), X(:), Y(:));
 */ //<>// //<>//
 //switch (type){
   if (type==1){ // nearest neighbor //<>// //<>// //<>//
     // make a source matrix filled with indeces and a border equal to 
     // kernel size
     int[][] kernel ={{-1,-1}, {-1, 0}, {-1, 1}, //<>// //<>//
                       {0,-1}, {0, 1},
                     {1,-1}, {1,0}, {1,1}};
     int border = 1; 
     int[][] ixm = makeIxMatrix(dim, dim, border);
     
     // every kernel movement adds a row to output topology
     int ctr = 0;
     for(int j=border; j<ixm.length-border; j++)
     {
       for(int i=border; i<ixm.length-border; i++)
       {
         for(int k=0; k<kernel.length; k++)
          {
            int ix = ixm[j+kernel[k][0]] [i+kernel[k][1]];
            // println("j= "+j+", i= " +i+ ", k= "+k+", ix= "+ix);
            if(ix>=0) retval[ctr][ix]=1; 
          }
          ctr++;
       }
     }
       
     
   } //<>// //<>// //<>// //<>// //<>// //<>// //<>//
   else
   {// TODO implement random
   }
 
 return retval; //<>// //<>// //<>//
}


int[][] makeIxMatrix(int x, int y, int border){
  int [][] retval = new int[y+2*border][x+2*border];
  int ctr=0;
  for (int[] row: retval)
    Arrays.fill(row, -1);
  //Arrays.fill(retval, -1);
  for(int j=border; j<retval.length-border; j++)
  {
    for(int i=border; i<retval[0].length-border; i++)
    {
      retval[j][i] = ctr;
      ctr++;
    }
  }
  return retval;
  
}

float[][] getColorGrid(PImage im, int col){
  /** Yield an array of r,g,b, or alpha values
  * im - image to process
  * col - 1=red, 2=green, 3=blue, 4=alpha
  */
  float [][] ret = new float[im.height][im.width];
  for(int j=0; j<ret.length; j++)
    for (int i=0; i<ret[0].length; i++){
    
      if (col ==1){
        ret[j][i] = red(im.get(i, j));
      } else if (col == 2){
        ret[j][i] = green(im.get(i, j));
      } else if (col == 3){
        ret[j][i] = blue(im.get(i, j));
      } else {
        ret[j][i] = alpha(im.get(i, j));
      }
    }
  return ret;
}

float[][] spikesToScalar(Buffer[] buffers, int rows, int cols){
  /*
  convert spikes from a population to scalars to be visualized
  */
  float maxspkval = 64;
  float [][] decod_act = new float[rows][cols];     
  for(int j=0; j<rows; j++)    
    for(int i=0; i<cols; i++){    
      float[] spk = buffers[i+rows*j].array();     
        
      decod_act[j][i] = 20*countSpikes(spk, maxspkval); 
      //println(decod_act[j][i]);      
    }  
  return decod_act;
}

float[] getCol(float[][] a, int col){
  float[] retval = new float[a.length];
  for(int j=0; j< a.length; j++)
    retval[j] = a[j][col];
  return retval;
}

float[][] copyMatrix(float[][] dest, float[][] m){
    float[][] retval = dest;
    for (int j = 0; j < m.length; j++) {
        for (int i = 0; i < m[0].length; i++) {
            retval[j][i] = m[j][i];
        }
    }
    return retval;
}
/**
 * Return indeces based on a threshold value
 ' m: matrix to evaluate
 * threshold: threshold which must be equalled or exceeded to yield inclusion
*/
int[][] getThresholdedIndeces(float[][] m, float threshold){
  IntList rows = new IntList();
  IntList cols = new IntList();
  for (int j = 0; j < m.length; j++) {
    for (int i = 0; i < m[0].length; i++) {
      if (m[j][i] >= threshold){
        rows.push(j);
        cols.push(i);
      }
    }
  }
  int[][] retval = new int[rows.size()][2];
  for(int j=0; j<retval.length; j++){
    retval[j][0] = rows.get(j);
    retval[j][1] = cols.get(j);
  }
  
  return retval;
}

/**
m - 1d array 
*/
int[] getThresholdedIndeces(float[] m, float threshold){
  IntList rows = new IntList();
  // IntList cols = new IntList();
  for (int j = 0; j < m.length; j++) {
    //for (int i = 0; i < m[0].length; i++) {
      if (m[j] >= threshold){
        rows.push(j);
        
      }
    //}
  }
  /*
  int[][] retval = new int[rows.size()][2];
  for(int j=0; j<retval.length; j++){
    retval[j][0] = rows.get(j);
    retval[j][1] = cols.get(j);
  }
  */
  
  return rows.array();
}

/**
top - a topology
indeces - indeces to find neighbors to
*/
int[] getNeighborIndeces(float[][] top, int[] indeces){
  IntList retval = new IntList();
  for(int i: indeces){
    float[] col = getCol(top, i);
    int[] neighbors = getThresholdedIndeces(col, 1);
    retval.append(neighbors);
  }
  return retval.array();
}

float[] map_array(float[] a, float from1 , float to1, float from2, float to2){
  float[] retval = zeros(a.length);
  for(int i=0; i< a.length; i++){
    retval[i] = map(a[i], from1, to1, from2, to2);
  }
  return retval;
}

float[] copyByIx(float[] a, int[] ix){
  float[] ret = zeros(a.length);
  for(int i: ix){
    ret[i] = a[i];
  }
  return ret;
}



float[][] arrayToMatrix(float[] a, int r, int c){
  float[][] ret = zeros(r, c);
  
  for (int j=0, row=0, col=0; j<a.length && row<r; j++){
     ret[row][col] = a[j];
     if (++col >= c){ col=0; row++;}
  }
  return ret;
}

float[] mask_array(float[] a, int[] keep){
  float[] retval = zeros(a.length);
  for(int i : keep){
    retval[i] = a[i];
  }
  return retval;
}

/**
*/
float[][] flipMatrixHor(float[][] m){
  float[][] ret = zeros(m.length, m[0].length);
  for (int j=0; j<ret.length; j++){
    for (int i=0; i<ret[0].length; i++){
      ret[j][i] = m[j][m[0].length-1-i];  
    }
  }

  return ret;
}

float[][] flipMatrixVer(float[][] m){
  float[][] ret = zeros(m.length, m[0].length);
  for (int j=0; j<ret.length; j++){
    for (int i=0; i<ret[0].length; i++){
      ret[j][i] = m[m.length-1-j][i];  
    }
  }

  return ret;
}

PImage pngBytesToImage(byte[] a){
 PImage ret = null;
 try {
        byte[] bytes = a;
        if (bytes == null) {
          return null;
        } else {
          //Image awtImage = Toolkit.getDefaultToolkit().createImage(bytes);
          Image awtImage = new ImageIcon(bytes).getImage();
 
          if (awtImage instanceof BufferedImage) {
            BufferedImage buffImage = (BufferedImage) awtImage;
            int space = buffImage.getColorModel().getColorSpace().getType();
            if (space == ColorSpace.TYPE_CMYK) {
              System.err.println("pngBytesToImage: bytes is a CMYK image, " +
                                 "only RGB images are supported.");
              return null;
              /*
              // wishful thinking, appears to not be supported
              // <a href="https://community.oracle.com/thread/1272045?start=0&tstart=0" target="_blank" rel="nofollow">https://community.oracle.com/thread/1272045?start=0&tstart=0</a>
              BufferedImage destImage =
                new BufferedImage(buffImage.getWidth(),
                                  buffImage.getHeight(),
                                  BufferedImage.TYPE_3BYTE_BGR);
              ColorConvertOp op = new ColorConvertOp(null);
              op.filter(buffImage, destImage);
              image = new PImage(destImage);
              */
            }
          }
 
          PImage image = new PImage(awtImage);
          if (image.width == -1) {
            System.err.println("pngBytesToImage: Bytes" +
                               " contains bad image data, or may not be an image.");
          }
 
          // if it's a .gif image, test to see if it has transparency
          //image.checkAlpha();
          
 
//          if (params != null) {
//            image.setParams(g, params);
//          }
          ret  = image;
        
      }
    } catch (Exception e) {
      // show error, but move on to the stuff below, see if it'll work
      e.printStackTrace();
    }
 
 return ret;
}

float[] blobToPoint(float[] a, float threshold){
  float[] retval = new float[a.length];
  int start=-1, stop=-1;
  for(int i=0; i<a.length; i++){
    if(a[i] > threshold){
      start = i;
      continue;
    }
  }
  for(int i=a.length-1; i>=0; i--){
    if(a[i] > threshold){
      stop = i;
      continue;
    }
  }
  if(start==stop && start >= 0 && stop >= 0) retval[start] = 1.0;
  else{
    int ix = (stop-start)/2 + start;
    if(ix >=0 && ix < retval.length) retval[ix] = 1.0;
  }
  return retval;
}

boolean isZero(float[] a, float tolerance){
  for(float i: a)
    if(i-tolerance > 0) return false;
  return true;
}

float hysteresis(float in, float prev, float lo_thr, float hi_thr){
  float r = -1.f;
  float hi = 1.f;
  float lo = 0.f;
  if(in  <= lo_thr) r = lo;
  else if(in >= hi_thr) r = hi;
  else r = prev;
  return r;
}

boolean isInside(float[] center, float radius, float[] check){
  boolean ret = false;
  
  boolean a = (check[0] >= (center[0] - radius));
  boolean b = (check[0] <= (center[0] + radius));
  boolean c = (check[1] >= (center[1] - radius));
  boolean d = (check[1] <= (center[1] + radius));
  // println("a: " + a + " b: " + b + " c: " +c + " d: " + d);
  return a && b && c && d;
  
}

boolean isInside(float[] center, float width, float depth, float[] check){
  boolean ret = false;
  
  boolean a = (check[0] >= (center[0] - width));
  boolean b = (check[0] <= (center[0] + width));
  boolean c = (check[1] >= (center[1] - depth));
  boolean d = (check[1] <= (center[1] + depth));
  // println("a: " + a + " b: " + b + " c: " +c + " d: " + d);
  return a && b && c && d;
  
}
