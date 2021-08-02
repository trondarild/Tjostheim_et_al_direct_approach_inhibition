//
//
import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;
import java.text.SimpleDateFormat;
import java.util.Date;
import oscP5.*;
import netP5.*;
import controlP5.*;
//import org.apache.commons.math3.util.MathUtils.normalizeAngle;

OscP5 oscP5;
ControlP5 cp5;

NetAddress myRemoteLocation;
String remoteaddr = "127.0.0.1";
FloatList data = new FloatList();
FloatList red, green, blue, depth;
ArrayList<BBox> borders = new ArrayList<BBox>();
ArrayList<BBox> obstacles = new ArrayList<BBox>();
ArrayList<BBox> barrierareas = new ArrayList<BBox>();
ArrayList<BBox> goals = new ArrayList<BBox>();
ArrayList<BBox> agents = new ArrayList<BBox>();
boolean border_semaphore = true;
boolean obst_semaphore = true;
boolean barrier_semaphore = true;
boolean goal_semaphore = true;
boolean agent_semaphore = true;
float far = 1000.f;
float turnrate = 10.f;
float fwdrate = 1.f;
PGraphics border_img;
PGraphics goal_img;
PGraphics agent_img;
int cam_dim = 24;
int wf_dim = 20;
float penweight = 0.015;
Map map;
float config_id; // sent from unity to keep track of used config
boolean firstrot = true;
boolean firsttrans = true;
float absAngle = 0;
float agentAngle = 0;
float alloAngle = radians(-90); // heading up, along negative y
PVector translation;
Queue<Float> rotQueue = new LinkedList<Float>(); 
float[][] red_matrix = null;
float[][] redstrip = new float[1][cam_dim]; 
String[] behaviour_str = {"No behaviour",
                          "Direct",
                          "Wavefront"};
float maxdepth = 0;
PFont font_8;
int popsize = 10;
float[] dirinputvec = ones(popsize);
float[][] onetoone_top = identity(popsize);
float direct_inhibition_fact = 10;
float noisefact = 1.f;

SpikingPopulation loomingPop;
SpikingPopulation collPop;
SpikingPopulation directPop;
SpikingPopulation wfPop;
// int rows, int cols, 
// float agrowth, float adecay, 
// float aaccumulate, float adecaythreshold)
LimitedLeakyIntegrator gateIntegrator = new LimitedLeakyIntegrator(
  1, 2, 0.1, 0.024, 0.9, 1, 1);
  
float goalradius = 5.f;
  
int approachTimer = 0;
Table statData = new Table();
boolean target_reached = false;
boolean ready = false;
boolean checking_newtrial = false;
int maxTrialLength = 1200; // max length of a single session before reset
int maxTrialsPerFactor = 5; // max number of sessions before changing inh factor
float[] inhFactors = {0, 0.1, 0.2, 0.4, 0.6, 0.8, 1.0}; //{0, 2, 4}; //,  , 4, 6, 8, 10}; 
int trialCtr = 0; // counts numer of sessions up to maxSessionPerFactor
int factorCtr = 0; // counts index of inh factors until length of inhFactors
boolean firstsession = true;

void setup(){
  size(800, 400);
  frameRate(10);
  randomSeed(100);

  loomingPop = new SpikingPopulation(
    "looming", popsize, NeuronType.eRegular_spiking, 10, noisefact);
  collPop = new SpikingPopulation(
    "collision", popsize,  NeuronType.eRegular_spiking, 10, noisefact);
  directPop = new SpikingPopulation(
    "direct", popsize, NeuronType.eRegular_spiking, 10, noisefact);
  wfPop = new SpikingPopulation(
    "wavefront", popsize, NeuronType.eRegular_spiking, 10, noisefact);
  //border_img = createGraphics(dim, dim);
  //goal_img = createGraphics(dim, dim);
  //agent_img = createGraphics(dim, dim);
  map = new Map(wf_dim, wf_dim);
  depth = new FloatList();
  red = new FloatList();
  green= new FloatList();
  blue = new FloatList();
  
  OscProperties op = new OscProperties();
  op.setListeningPort(12001);
  op.setDatagramSize(9200);
  oscP5 = new OscP5(this, op);
  myRemoteLocation = new NetAddress(remoteaddr,12002);
  
  cp5 = new ControlP5(this);
  font_8 = createFont("Arial", 8);

  // session data saved to csv
  statData.addColumn("id");
  statData.addColumn("time");
  statData.addColumn("strategy");
  statData.addColumn("x");
  statData.addColumn("y");
  statData.addColumn("inh");
  statData.addColumn("config");
  statData.addColumn("trial");
  statData.addColumn("barrier");

  
  
  

  rectMode(CENTER); 
}

void draw(){
  background(51);
  if(!ready||checking_newtrial){
    pushMatrix();
    text("waiting for ready-signal", 100, 100);
    popMatrix();
    return;
  }
  // depth and egocentric grid
  float[][] depth_matrix;
  float collision_fact = 0;
  float looming_fact = 0;
  float[] coll_weight = {1, 0.5, 0.25};

  // show session info
  pushMatrix();
  translate(239,40);
  text("Trial: " +  trialCtr, 0, 0);
  text("Factor ix: " + factorCtr, 0, 12);
  text("Inh factor: " + inhFactors[factorCtr], 0, 24);
  popMatrix();
  
  if(depth.size()==cam_dim*cam_dim){
    depth_matrix = flipMatrixHor(
      arrayToMatrix(depth.array(), cam_dim, cam_dim));
    pushMatrix();
    translate(339,45);
    //scale(1);
    drawColGrid(0, 0, 5, 0, "Depth", multiply(2000, depth_matrix));
    //drawColGrid(0, 0, 10, multiply(2000, depth_matrix));
    popMatrix();
    //printMatrix("depth", depth_matrix);
    float[][] depthmap = depthToGrid(depth_matrix[cam_dim/2], cam_dim, far);
    //printMatrix("depthmap first", depthmap);
    pushMatrix();
    translate(486, 46);
    drawColGrid(0, 0, 5, 0, "Egocentric Grid", multiply(200, depthmap));
    popMatrix();

    // use depthmap to construct collision detection field for 
    // informing change of approach strategy
    float[][] collisionfield = {
      multiply(coll_weight[0], depthmap[1]), 
      multiply(coll_weight[1], depthmap[2]), 
      multiply(coll_weight[2], depthmap[3]) };
    collision_fact = sumMatrix(collisionfield);

    
    
    
  } 
  // show collision fact
  pushMatrix();
  translate(486, 200);
  text("Collision factor: " + collision_fact, 0, 0);
  popMatrix();
   
  // Red (colour vision), red strip, redstrip pt
  if(red.size()==cam_dim*cam_dim){
    
    red_matrix = flipMatrixHor(arrayToMatrix(red.array(), cam_dim, cam_dim));
    pushMatrix();
    
    translate(37,229);
    // rotate(PI);
    scale(0.8);
    drawColGrid(0, 0, 8,0, "Red", multiply(400, red_matrix));
    //drawColGrid(0, 0, 10, multiply(200, red_matrix));
    popMatrix();
    
      redstrip[0] = red_matrix[cam_dim/2];
      
    pushMatrix();
    translate(242,313);
    scale(1);
    drawColGrid(0, 0, 10,0, "Red_strip", multiply(400, redstrip));
    popMatrix();
    
    float[][] pointstrip = {blobToPoint(redstrip[0], 0.1)};
    pushMatrix();
    translate(242,353);
    scale(1);
    drawColGrid(0, 0, 10,0, "Red_strip_pt", multiply(400, pointstrip));
    popMatrix();
    //println("redstrip min: " + min(redstrip[0]) + "; max: " + max(redstrip[0]));
    //looming_fact = sumArray(range_expand(0.11, 0.5, 0, 1, redstrip[0]));
    looming_fact = sumArray(limitval(0, 1, multiply(10, subtract(redstrip[0], 0.11))));
    
    
  }
  // show looming factor
  pushMatrix();
  translate(486, 216);
  text("Looming factor: " + looming_fact, 0, 0);
  popMatrix();
  
  // draw grid
  stroke(200);
  strokeWeight(0.1);
  
  
  pushMatrix();
  translate(30,14);
  scale(1);
  //image(border_img, 0, 0);
  float[][] tmpmap = map(map.getFMap(), 0, 255, 50, 255 );
  drawColGrid(0,0,10,0, "grid", tmpmap); //multiply(50, map.getFMap()));
  popMatrix();
  //updateMapFromImage(border_img, goal_img, agent_img, map);
  //int dir = map.propagateWavefront();
  
  int dir = updateWavefront(borders, obstacles, 
    goals, agents, wf_dim, penweight, map);
  
  
  
  
  //float angle = rotQueue.peek() != null ? rotQueue.poll():0.f;
  alloAngle = absAngle; // normalizeAngle(alloAngle+angle); 
  //println("alloangle=" + alloAngle);
  //alloDirToEgo(dir, alloAngle);
  float[] motor_grid = alloDirToEgo(dir, alloAngle, map, fwdrate, turnrate);
  //printArray("motorgrid", motor_grid);
  float transl_fact = 100.f;
  float[] tmp = {transl_fact * sumArray(motor_grid) > 0 ? 20 : 0};
  
  // ===neural selection/decision===

  float[] dirinput = multiply(transl_fact * looming_fact, dirinputvec);
  loomingPop.setDirect(dirinput);
  loomingPop.tick();
  dirinput = multiply(transl_fact * collision_fact, dirinputvec);
  collPop.setDirect(dirinput);
  collPop.tick();
  
  //directPop.setDirect(tmp);
  directPop.excite(loomingPop.getOutput(), onetoone_top);
  direct_inhibition_fact = inhFactors [factorCtr];
  for(int i=0; i<(int)direct_inhibition_fact; i++){
    directPop.inhibit(collPop.getOutput(), onetoone_top);
  }
  //directPop.inhibit(collPop.getOutput(), multiply(direct_inhibition_fact, onetoone_top));
  directPop.tick();

  //wfPop.setDirect(tmp);
  wfPop.excite(collPop.getOutput(), onetoone_top);
  wfPop.tick();
  //println("wfpop: ", sumArray( wfPop.getBuffers()[0].array()));
  float normdirect = 1.f/popsize * sumArray(directPop.getNormOutput());
  float normwf = 1.f/popsize * sumArray(wfPop.getNormOutput());
  float[][] integr_input = 
    {{normdirect, normwf}};
  gateIntegrator.setInput(integr_input);
  gateIntegrator.tick();

  float[] gate_fact_raw = {0.01, looming_fact, collision_fact}; // no inh, only strength
  float[] gate_fact = {
    0.01, 
    gateIntegrator.getOutput()[0][0], 
    gateIntegrator.getOutput()[0][1]};
  // === end neuro ===
  
  pushMatrix();
  translate(486, 300);
  float yscl = 47.8;
  float xscl = 134.0;
  float ymax = 7.f;
  barchart_array(
    gate_fact_raw,
    0, 0,
    yscl, xscl,
    #A0EA9A,
    1,
    ymax,
    "argmax gate",
    font_8, font_8
  );
  popMatrix();

  pushMatrix();
  translate(486, 370);
  
  ymax = 1.f;
  barchart_array(
    gateIntegrator.getOutput()[0],
    0, 0,
    yscl, xscl,
    #A0EA9A,
    1,
    ymax,
    "neuro gate",
    font_8, font_8
  );
  popMatrix();

  float[] motor_direct = behaviourFromDirection(redstrip[0], turnrate, fwdrate);
  
  float[][] motor = {
    {noise((float)random(1)),
    noise((float)random(1)),0}, 
    motor_direct, 
    motor_grid};
    
  int motor_select = argmax(gate_fact);
  TableRow newRow = statData.addRow();
  if(agents.size() > 0){
    newRow.setInt("id", statData.getRowCount()-1);
    newRow.setInt("time", approachTimer);
    newRow.setInt("strategy", motor_select); 
    newRow.setInt("trial", trialCtr);
    newRow.setFloat("x", agents.get(0).pos.x);
    newRow.setFloat("y", agents.get(0).pos.y);
    newRow.setFloat("inh", direct_inhibition_fact);
    newRow.setFloat("config", config_id);

    boolean inBarrierArea = false;
    float scalefact = 0.5;
    for(int i =0; i<barrierareas.size() && !inBarrierArea; i++){
      inBarrierArea = isInside(
        barrierareas.get(i).pos.array(),
        barrierareas.get(i).sz.x*scalefact,
        barrierareas.get(i).sz.y*scalefact,
        agents.get(0).pos.array()
      );
    }
    newRow.setInt("barrier", inBarrierArea ? 1 : 0);
    pushMatrix();
    translate(239,200);
    text("Barrier: " + inBarrierArea, 0, 0);
    popMatrix();
    
    


  }
  if(true){
    goLeft(motor[motor_select][0]);
    goRight(motor[motor_select][2]);
    goFwd(motor[motor_select][1]);
  }
  pushMatrix();
  translate(242, 28);
  text(behaviour_str[motor_select], 0,0);
  popMatrix();
  
  // draw spiking behaviour
  pushMatrix();
    translate(280, 80);
    text("lom", -40, 0);
      pushMatrix();
      scale(1, 0.25);
      drawSpikeStrip(loomingPop.getBuffers(), 30f);
      popMatrix();
    translate(0, 25);
    text("dir", -40, 0);
      pushMatrix();
      scale(1, 0.25);
      drawSpikeStrip(directPop.getBuffers(), 30f);
      popMatrix();
    translate(0, 25);
    text("col", -40, 0);
      pushMatrix();
      scale(1, 0.25);
      drawSpikeStrip(collPop.getBuffers(), 30f);
      popMatrix();
    translate(0, 25);
    text("wf", -40, 0);
      pushMatrix();
      scale(1, 0.25);
      drawSpikeStrip(wfPop.getBuffers(), 30f);
      popMatrix();
  popMatrix();
  
  
  //if(dir == map.NOTHING && !target_reached){
  /*  */
  boolean inside = goals.size() > 0 ? isInside(
    goals.get(0).pos.array(),
    goals.get(0).sz.x,
    agents.get(0).pos.array()) : false;
    
  //if(inside && dir == 0 && target_reached==false && approachTimer > 10){
  if(inside && approachTimer > 10){
    
    // reached goal
    targetReached(approachTimer, statData, "test");
    target_reached = true;
    
  
  } 
  else if(approachTimer == maxTrialLength ){
    println("maxTrialLength reached, resetting");
    reset();
  }
  else{ 
    approachTimer++;
    //println("dir: " + dir + "; target reached: " + target_reached);
  }

  
}

int updateWavefront(ArrayList<BBox> aborders,
                    ArrayList<BBox> aobstacles,
                    ArrayList<BBox> agoals,
                    ArrayList<BBox> aagents,
                    int adim, float apenweight, 
                    Map amap){
  PGraphics border_img = createGraphics(adim, adim);
  PGraphics obst_img = createGraphics(adim, adim);
  PGraphics goal_img = createGraphics(adim, adim);
  PGraphics agent_img = createGraphics(adim, adim);
  
  float[] upperleft = getAggregateUpperLeft(aborders);
  
  border_img.beginDraw();
  border_img.rectMode(CENTER);
  border_img.translate(-upperleft[0], -upperleft[1]);
  //border_img.translate(-dim/2, -dim/2);
  //border_img.rotate(radians(90));
  //border_img.translate(dim/2, dim/2);
  //border_img.translate(-dim/2, -dim/2);

  border_img.background(color(0,0,0,255));
  
  obst_img.beginDraw();
  obst_img.rectMode(CENTER);
  obst_img.translate(-upperleft[0], -upperleft[1]);
  obst_img.background(color(0,0,0,255));
  
  goal_img.beginDraw();
  goal_img.rectMode(CENTER);
  goal_img.translate(-upperleft[0], -upperleft[1]);
  goal_img.background(color(0,0,0,255));
  
  agent_img.beginDraw();
  agent_img.rectMode(CENTER);
  agent_img.translate(-upperleft[0], -upperleft[1]);
  agent_img.background(color(0,0,0,255));
  
  border_img.strokeWeight(apenweight);
  obst_img.strokeWeight(apenweight);
  goal_img.strokeWeight(apenweight);
  agent_img.strokeWeight(apenweight);
  
  if(border_semaphore){
    border_semaphore = false;
    drawObjList(aborders, 140, 140, border_img);
    border_semaphore = true;
  }
  if(obst_semaphore){
    obst_semaphore = false;
    drawObjList(aobstacles, 200, 200, border_img);
    obst_semaphore = true;
  }
  if(goal_semaphore){
    goal_semaphore = false; // #DE1616
    drawObjList(agoals, 100, 100, goal_img);
    goal_semaphore = true;
  }
  if(agent_semaphore){
    agent_semaphore = false; // #DE1616
    drawObjList(aagents, 100, 100, agent_img);
    agent_semaphore = true;
  }
  
  border_img.endDraw();
  goal_img.endDraw();
  agent_img.endDraw();
 

  updateMapFromImage(border_img, goal_img, agent_img, amap);
  int dir = amap.propagateWavefront();
  return dir;
}

void drawObjList(ArrayList<BBox> lst, 
  color strk, color fill, PGraphics g){
  g.pushStyle();
  g.stroke(strk);
  g.fill(fill);
  g.strokeWeight(0.5);
  for(BBox b: lst){
    g.pushMatrix();
    g.translate(b.pos.x, b.pos.y);
    g.rect(0, 0, b.sz.x, b.sz.y);
    g.popMatrix();
  }
  g.popStyle();
}

Map updateMapFromImage(PImage border, PImage goal, PImage agent, Map mp){
  border.loadPixels();
  goal.loadPixels();
  agent.loadPixels();

  int ctr = 0;
  for(int j=0; j<mp.getSizeX() && border.pixels.length > 0; j++){ // x is rows, y is cols in map{
    for(int i=0; i<mp.getSizeY(); i++){
      int val = mp.NOTHING;
      if(red(border.pixels[ctr]) > 0) val = mp.WALL;
      else if(red(goal.pixels[ctr]) > 0) val = mp.GOAL;
      else if(red(agent.pixels[ctr]) > 0) val = mp.ROBOT;
      mp.placeValue(j, i, val);
      ctr++;
    }
  }
  return mp;
}

void oscEvent(OscMessage theOscMessage) {

  /* print the address pattern and the typetag of the received OscMessage */
  String adrpattern = theOscMessage.addrPattern();
  //print("### received an osc message.");
  //println(" addrpattern: "+adrpattern);
  //println(" typetag: "+theOscMessage.typetag());
  
  //String typetag = theOscMessage.typetag();
  
  //typetag = theOscMessage.typetag();
  
  data.clear();
  Object[] args = theOscMessage.arguments();
  if(adrpattern.equals("/ready")){
    ready = true;
    if(firstsession && approachTimer > 1)
      firstsession = false;
    else if (!firstsession && approachTimer > 100)
      newTrial();
    println();
    println("== received ready ==");
    println();
  } else {
  for(int i=args.length-1; i>=0; i--){
      //println((float)args[i]);
      data.append((float)args[i]);
  }
  if(adrpattern.equals("/left/camera_r")){
    red = data.copy();
    float[] tmparray = subtract(red.array(), green.array());
    red = new FloatList(
      limitval(0.1, 1.2, 
        subtract(tmparray, blue.array())));
    //println("red len: " + red.size());
    //red.reverse();
  }
  else if(adrpattern.equals("/left/camera_g")){
    green = data.copy();
    green = new FloatList(divide(green.array(), 1.0));
    //red.reverse();
  }
  else if(adrpattern.equals("/left/camera_b")){
    blue = data.copy();
    blue = new FloatList(divide(blue.array(), 4.2));
    //red.reverse();
  }
  else if (adrpattern.equals("/left/depth/camera")){
    depth = data.copy();
    //depth.reverse();
    //println("depth copied length: " + depth.size());
  }
  else if(adrpattern.equals("/borders/") && border_semaphore){
    border_semaphore = false;
    processOscBBoxList(borders, data);
    border_semaphore = true;
    
  }
  else if(adrpattern.equals("/obstacles/") && obst_semaphore){
    obst_semaphore = false;
    processOscBBoxList(obstacles, data);
    obst_semaphore = true;
    
  }
  else if(adrpattern.equals("/barrierareas/") && barrier_semaphore){
    barrier_semaphore = false;
    processOscBBoxList(barrierareas, data);
    barrier_semaphore = true;
    
  }
  else if(adrpattern.equals("/goals/") && goal_semaphore){
    goal_semaphore = false;
    processOscBBoxList(goals, data);
    goal_semaphore = true;
    //printArray("goals: ", data.array());
    
  }
  else if(adrpattern.equals("/agents/") && agent_semaphore){
    agent_semaphore = false;
    //println("agents: ", data.array().length);
    processOscBBoxList(agents, data);
    agents.get(0).sz.x = 1;
    agents.get(0).sz.y = 1;
    agents.get(0).pos.x -= 0;
    agents.get(0).pos.y -= 0;
    //println("goals " + agents.get(0).toString());
    agent_semaphore = true;
    
  }
  else if(adrpattern.equals("/config/")) {
    config_id = data.get(0);
  }
  else if(adrpattern.equals("/camera/rotation")){
    //printArray("rotation", data.array());  
    if(!firstrot){
      agentAngle = radians(data.array()[0]); // unity sends degrees
      rotQueue.add(agentAngle);
      //println(rotQueue.toString());
    }
    firstrot = false;
  }
  else if(adrpattern.equals("/camera/absrotation")){
    //printArray("rotation", data.array());  
    if(!firstrot){
      absAngle = radians(data.array()[0]); // unity sends degrees
      
      //println("absangle: " + absAngle);
    }
    firstrot = false;
  }
  }
  
}

ArrayList<BBox> processOscBBoxList(ArrayList<BBox> lst, FloatList data){
  lst.clear();
  // get tuple of 3d: pos and bbox: 6 floats
  //println("goals: ", data.array().length);
  // borders contains 4 floats: x y w h
  for(int i=0; i<data.array().length; i+=4){
    int ctr = i;
    BBox box = new BBox();
    box.sz.x = 2*data.array()[ctr++];
    box.sz.y = 2*data.array()[ctr++];
    box.pos.x = data.array()[ctr++];
    box.pos.y = data.array()[ctr++];
    //println("goals " + box.toString());
    lst.add(box);
  }
  return lst;
}

class BBox{
 public PVector pos;
 public PVector sz;
 public float rot;
 
 BBox(){
   pos = new PVector();
   sz = new PVector();
 }
 public String toString(){
    String ret = "pos: " + pos.x + ", " + pos.y;
    ret += "; sz: " + sz.x + ", " + sz.y;
    return ret;
 }
}

public void Right(int val){
  goRight(turnrate);
}

public void Left(int val){
  goLeft(turnrate); 
}

public void Fwd(int val){
  goFwd(fwdrate); 
}

public void Bkw(int val){
  goBkw(fwdrate); 
}

//
//
public void goRight(float val){
  OscMessage myMessage = new OscMessage("/rotate_right");
  
  myMessage.add(val); /* add an int to the osc message */

  /* send the message */
  oscP5.send(myMessage, myRemoteLocation); 
}

public void goLeft(float val){
  OscMessage myMessage = new OscMessage("/rotate_left");
  
  myMessage.add(val); /* add an int to the osc message */

  /* send the message */
  oscP5.send(myMessage, myRemoteLocation); 
}

public void goFwd(float val){
  OscMessage myMessage = new OscMessage("/forward");
  
  myMessage.add(val); /* add an int to the osc message */

  /* send the message */
  oscP5.send(myMessage, myRemoteLocation); 
}

public void goBkw(float val){
  OscMessage myMessage = new OscMessage("/backward");
  
  myMessage.add(val); /* add an int to the osc message */

  /* send the message */
  oscP5.send(myMessage, myRemoteLocation); 
}

public void reset(){
  OscMessage myMessage = new OscMessage("/reset");
  
  myMessage.add(1.f); /* add an int to the osc message */

  /* send the message */
  oscP5.send(myMessage, myRemoteLocation); 

  // reset networks
  loomingPop.reset();
  collPop.reset();
  directPop.reset();
  wfPop.reset();
  gateIntegrator.reset();
  // create a new individual each reset
  loomingPop.randomize();
  collPop.randomize();
  directPop.randomize();
  wfPop.randomize();


  target_reached = false;
  ready = false;

  stopWorld();

  
  
}

public void stopWorld(){
  OscMessage myMessage = new OscMessage("/stop");
  
  myMessage.add(1.f); /* add an int to the osc message */

  /* send the message */
  oscP5.send(myMessage, myRemoteLocation); 

  // reset networks
  // loomingPop.reset();
  // collPop.reset();
  // directPop.reset();
  // wfPop.reset();
  // gateIntegrator.reset();
  // target_reached = false;
  // ready = false;
  
}

public void quitWorld(){
  OscMessage myMessage = new OscMessage("/quit");
  
  myMessage.add(1.f); /* add an int to the osc message */

  /* send the message */
  oscP5.send(myMessage, myRemoteLocation); 

  // reset networks
  // loomingPop.reset();
  // collPop.reset();
  // directPop.reset();
  // wfPop.reset();
  // gateIntegrator.reset();
  // target_reached = false;
  // ready = false;
  
}

void newTrial(){
  checking_newtrial = true;
  // save checkpoint
  saveTable(statData, "data/temp/"
     + "data_"
     + "trial=" + trialCtr 
     + "factor=" + inhFactors[factorCtr] 
     + "popsz=" + popsize
     + "_temp.csv");

  if(ready){
    // re-randomize populations to create a new individual
    
    maxdepth=0;
    approachTimer = 0;
    trialCtr++;
    if (trialCtr == maxTrialsPerFactor){
      
      trialCtr = 0;
      factorCtr++;
      println("New factor session; factor ix = " + factorCtr);
      if(factorCtr == inhFactors.length)
        sessionEnd(statData, 
          "stats_" 
          + "configid=" + config_id + "_"
          + "popsz=" + popsize + "_"
          + "time=" + getCurrentTimeStamp());
    }
    else 
      println("New trial; trialctr = " + trialCtr);
  }
  checking_newtrial = false;
}

float[][] graphicsToMatrix(PGraphics g){
  float[][] ret = zeros(g.height, g.width);
  g.loadPixels();
  int ctr = 0;
  for(int r=0; r<ret.length; r++){
    for(int c=0; c<ret[0].length; c++){
      int val = g.pixels[ctr++];
      float gr = green(val);
      float rd = red(val);
      float bl = blue(val);
      float al = alpha(val);
      float sum = gr + rd + bl;
      // print(rd+", "+gr+", "+bl+", "+al+"; ");
      ret[r][c] = sum == 0 ? 0.f : 1.f;
    }
  }
  return ret;
}

// translate allo - hack: 
void alloDirToEgo(int dir, float heading){
  
  
  
  //float normheading = normalizeAngle(heading);
  PVector hd = PVector.fromAngle(heading);
  PVector upv = PVector.fromAngle(radians(-90));
  PVector rv = PVector.fromAngle(radians(0));
  PVector dnv = PVector.fromAngle(radians(90));
  PVector lv = PVector.fromAngle(radians(180));
  float delta = 0.5f;
  float fwd_frac = 0.1;
  
  if(dir == map.DOWN) {
    //println("down diff: " + PVector.angleBetween(hd, dnv));
    //println("signed rel angle: " + signedRelAngle(hd, dnv)); 
   if(PVector.angleBetween(hd, dnv) < delta) 
      goFwd(fwdrate);
    else if(signedRelAngle( hd, dnv) > 0) {goRight(turnrate);goFwd(fwd_frac*fwdrate);}
    else {goLeft(turnrate); goFwd(fwd_frac*fwdrate);}
   
   
  }
  else if (dir == map.UP){
    // placeValue(rX-1, rY, ROBOT);
    //goRight(turnrate);
    //println("up diff: " + PVector.angleBetween(hd, upv));
    //println("signed rel angle: " + signedRelAngle(hd, upv)); 
    if(PVector.angleBetween(hd, upv) < delta) 
      goFwd(fwdrate);
    else if (signedRelAngle(hd, upv) > 0) {goRight(turnrate);goFwd(fwd_frac*fwdrate);}
    else {goLeft(turnrate);goFwd(fwd_frac*fwdrate);}
  }
  else if (dir == map.RIGHT) {
    //println("right diff: " + PVector.angleBetween(hd, rv));
    //println("signed rel angle: " + signedRelAngle(hd, rv));
    //placeValue(rX, rY+1, ROBOT);
    if(PVector.angleBetween(hd, rv) < delta) 
      goFwd(fwdrate);
    else if(signedRelAngle(hd, rv) > 0) {goRight(turnrate);goFwd(fwd_frac*fwdrate);}
    else {goLeft(turnrate);goFwd(fwd_frac*fwdrate);}
    //goFwd(0.5*fwdrate);
  }
  else if (dir == map.LEFT) {
    // placeValue(rX, rY-1, ROBOT);
    //println("left diff: " + PVector.angleBetween(hd, lv));
    //println("signed rel angle: " + signedRelAngle(hd, lv)); 
    if(PVector.angleBetween(hd, lv) < delta) 
      goFwd(fwdrate);
    else if(signedRelAngle(hd, lv) > 0) {goRight(turnrate);goFwd(fwd_frac*fwdrate);}
    else {goLeft(turnrate);goFwd(fwd_frac*fwdrate);}
  }
}

float[] alloDirToEgo(int dir, float heading, Map map, float fwdrate, float turnrate){
  float[] retval = zeros(3);
  int turn_left = 0, fwd = 1, turn_right =2;

  //float normheading = normalizeAngle(heading);
  PVector hd = PVector.fromAngle(heading);
  PVector upv = PVector.fromAngle(radians(-90));
  PVector rv = PVector.fromAngle(radians(0));
  PVector dnv = PVector.fromAngle(radians(90));
  PVector lv = PVector.fromAngle(radians(180));
  float delta = 0.5f;
  float fwd_frac = 0.1;
  
  if(dir == map.DOWN) {
    //println("down diff: " + PVector.angleBetween(hd, dnv));
    //println("signed rel angle: " + signedRelAngle(hd, dnv)); 
   if(PVector.angleBetween(hd, dnv) < delta) 
      //goFwd(fwdrate);
      retval[fwd] = fwdrate;
    else if(signedRelAngle( hd, dnv) > 0) {
      // goRight(turnrate);goFwd(fwd_frac*fwdrate);
      retval[turn_right] = turnrate;
      retval[fwd] = fwd_frac*fwdrate;
    }
    else {
      // goLeft(turnrate); goFwd(fwd_frac*fwdrate);
      retval[turn_left] = turnrate;
      retval[fwd] = fwd_frac*fwdrate;
    }
  }
  else if (dir == map.UP){
    // placeValue(rX-1, rY, ROBOT);
    //goRight(turnrate);
    //println("up diff: " + PVector.angleBetween(hd, upv));
    //println("signed rel angle: " + signedRelAngle(hd, upv)); 
    if(PVector.angleBetween(hd, upv) < delta) 
      //goFwd(fwdrate);
      retval[fwd] = fwdrate;
    else if (signedRelAngle(hd, upv) > 0) {
      // goRight(turnrate);goFwd(fwd_frac*fwdrate);
      retval[turn_right] = turnrate;
      retval[fwd] = fwd_frac*fwdrate;
    }
    else {
      // goLeft(turnrate); goFwd(fwd_frac*fwdrate);
      retval[turn_left] = turnrate;
      retval[fwd] = fwd_frac*fwdrate;
    }
  }
  else if (dir == map.RIGHT) {
    //println("right diff: " + PVector.angleBetween(hd, rv));
    //println("signed rel angle: " + signedRelAngle(hd, rv));
    //placeValue(rX, rY+1, ROBOT);
    if(PVector.angleBetween(hd, rv) < delta) 
      //goFwd(fwdrate);
      retval[fwd] = fwdrate;
    else if(signedRelAngle(hd, rv) > 0) {
        // goRight(turnrate);goFwd(fwd_frac*fwdrate);
      retval[turn_right] = turnrate;
      retval[fwd] = fwd_frac*fwdrate;
    }
    else {
      // goLeft(turnrate); goFwd(fwd_frac*fwdrate);
      retval[turn_left] = turnrate;
      retval[fwd] = fwd_frac*fwdrate;
    }
    //goFwd(0.5*fwdrate);
  }
  else if (dir == map.LEFT) {
    // placeValue(rX, rY-1, ROBOT);
    //println("left diff: " + PVector.angleBetween(hd, lv));
    //println("signed rel angle: " + signedRelAngle(hd, lv)); 
    if(PVector.angleBetween(hd, lv) < delta) 
      //goFwd(fwdrate);
      retval[fwd] = fwdrate;
    else if(signedRelAngle(hd, lv) > 0) {
        // goRight(turnrate);goFwd(fwd_frac*fwdrate);
      retval[turn_right] = turnrate;
      retval[fwd] = fwd_frac*fwdrate;
    }
    else {
      // goLeft(turnrate); goFwd(fwd_frac*fwdrate);
      retval[turn_left] = turnrate;
      retval[fwd] = fwd_frac*fwdrate;
    }
  }
  retval[fwd] = min(retval[fwd], fwdrate); // should not exceed fwd rate
  return retval;
}

float normalizeAngle(float theta){
  return (float)(theta - TWO_PI * Math.floor((theta + Math.PI) / TWO_PI));
}

float[] behaviourFromDirection(float[] direction, 
  float aturnrate, float afwdrate){
    
  float retval[] = zeros(3);
  int turn_left = 0, fwd = 1, turn_right =2;
  float minrate = 0.5;
  // TODO replace with neurons
  int left_stp = direction.length/2;
  int left_srt = 0;
  int right_stp = direction.length-1;
  int right_srt = left_srt + 1;
  if(isZero(direction, 0.01)) 
    retval[turn_right] = minrate*aturnrate;
  else {
    int dir = argmax(direction);
    if(dir >= left_srt && dir < left_stp){
      retval[turn_left] = turnrate;
      retval[fwd] = minrate*afwdrate;
    } else if(dir >right_srt && dir <= right_stp) {
      retval[turn_right] = aturnrate;
      retval[fwd] = minrate*afwdrate;
    
    } else if(dir == left_stp || dir == right_srt){
      retval[fwd] = afwdrate;
    }
  }
  return retval;
}

float logDepthToLinear(float logval, float far){
  // https://stackoverflow.com/questions/18182139/logarithmic-depth-buffer-linearization
  float C = 1.f;
  return (exp(logval*log(C*far+1)) - 1)/C;
}


float[][] depthToGrid(float[] depthstrip, 
  int rows, float farpl){
  
  
  float[][] retval = zeros(rows, depthstrip.length);
  float currentmax = max(depthstrip);
  
  float far = farpl; // far place
  // TODO remove global var
  maxdepth = max(currentmax, maxdepth);
  float localmax = logDepthToLinear(maxdepth, far); 
  int maxcol = depthstrip.length;
  for(int c = 0; c<depthstrip.length; c++){
    float z = logDepthToLinear(depthstrip[c], far);
    int row = (int)map(z, 0, localmax, 0, rows-1);
    if(row>0){
    //if(row<rows-1)
      // mirror to have viewpoint looking south
      int tgt_col = maxcol - (c+1);
      //retval[max(0,row-2)][tgt_col] = 1.f;
      //retval[max(0,row-1)][tgt_col] = 1.f;
      retval[row][tgt_col] = 1.f;
    }
    
  }
 
  return retval;
}

void targetReached(int timer, Table tab, String fname){
  // send reset'
  println();
  println("== target reached sending reset ==");
  println();
  reset();
  ready = false;
}

void sessionEnd(Table tab, String fname){
  saveTable(tab, "data/" + fname + "_final.csv");
  println("== session ended, saving data: " + fname + " ==");
  System.exit(0);
}

float[] getAggregateUpperLeft(ArrayList<BBox> lst){
  float[] mn = {
    lst.size() > 0 ? lst.get(0).pos.x:0, 
    lst.size() > 0 ? lst.get(0).pos.y:0};  
  for(BBox b: lst){
    mn[0] = min(mn[0], b.pos.x);
    mn[1] = min(mn[1], b.pos.y);  
  }
  return mn;
}

float[] getAggregateCenter(ArrayList<BBox> lst){
  float[] mn = {
    lst.size() > 0 ? lst.get(0).pos.x:0, 
    lst.size() > 0 ? lst.get(0).pos.y:0};
  float[] mx = {
    lst.size() > 0 ? lst.get(0).pos.x:0, 
    lst.size() > 0 ? lst.get(0).pos.y:0};
  float[] ret = zeros(2);
  
  for(BBox b: lst){
    mn[0] = min(mn[0], b.pos.x);
    mn[1] = min(mn[1], b.pos.y);
    mx[0] = max(mx[0], b.pos.x);
    mx[1] = max(mx[1], b.pos.y);
  }
  ret[0] = mn[0] + 0.5*(mx[0]-mn[0]);
  ret[1] = mn[1] + 0.5*(mx[1]-mn[1]);
  //ret = mx;
  //ret = divide(ret, 2);
  return ret;
}

public String getCurrentTimeStamp() {
    return new SimpleDateFormat("yyyy-MM-dd HH:mm:ss").format(new Date());
}
