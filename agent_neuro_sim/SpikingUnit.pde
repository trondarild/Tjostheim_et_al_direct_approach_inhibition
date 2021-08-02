import java.util.Arrays;
//import java.util.stream.FloatStream;
/* Single spiking unit
*/
//enum NeuronType {eRegular_spiking, eIntrinsically_bursting, 
//                 eChattering, eFast_spiking, eLow_threshold, eResonator}
enum NeuronType {  eIntrinsically_bursting,eChattering,eFast_spiking,
  eLow_threshold,eResonator,eRegular_spiking}
  
class SpikingUnit{
  
                 
  int population_size=1;
  boolean debug = false;
  float max_threshold = 65.0;
  int substeps;
  NeuronType neuronType;
  float tau_recovery;
  float coupling;
  float reset_voltage;
  float reset_recovery;
  float dir_current;
  float voltage;
  float recovery;
  FloatList e_synapse;
  FloatList i_synapse;
  String name;
  float noisefact = 0;
  
  //float[] vlt_buffer;
  // constructor
  SpikingUnit(String aname, NeuronType a_ntype,
              int a_substeps){
    name = aname;            
    substeps = a_substeps;
    e_synapse = new FloatList();
    i_synapse = new FloatList();
    noisefact = 0;
    // vlt_buffer = new float[a_buffersz];
    setNeuronType(a_ntype);
   
  }
  
  SpikingUnit(String aname, NeuronType a_ntype,
              int a_substeps, float noise){
    name = aname;            
    substeps = a_substeps;
    e_synapse = new FloatList();
    i_synapse = new FloatList();
    noisefact = noise;
    // vlt_buffer = new float[a_buffersz];
    setNeuronType(a_ntype);
    randomize();
   
  }
  
  // accessors
  float getVlt(){ return voltage; }
  float getOutput(){return voltage;}
  float getNormOutput(){return voltage >= max_threshold? 1.f : 0.f; }
  float getPhase(){return map(voltage, reset_voltage, max_threshold, 0, TWO_PI);}
  float getResetVoltage(){return reset_voltage;}
  float getTauRecovery(){return tau_recovery;}
  float getCoupling(){return coupling;}
  float getResetRecovery(){return reset_recovery;}
  
  void setDirect(float current){ dir_current = current; }
  void setDbg(boolean on){ debug = on; }
  void modulateResetVoltage(float val){reset_voltage += val;}
  void setResetVoltage(float val){reset_voltage = val;}
  void setNeuronType(NeuronType a_ntype){
     
     switch(a_ntype){
      case eIntrinsically_bursting:
          tau_recovery = 0.02f;
          coupling = 0.2f;
          reset_voltage = -55;
          reset_recovery = 4;
          break;
      case eChattering:
          tau_recovery = 0.02f;
          coupling = 0.2f;
          reset_voltage = -50;
          reset_recovery = 2;
          break;
      case eFast_spiking:
          
          tau_recovery = 0.02f;
          coupling = 0.25f;
          reset_voltage = -65;
          reset_recovery = 2;

          break;
      case eLow_threshold:
          tau_recovery = 0.1f;
          coupling = 0.25f;
          reset_voltage = -65.f;
          reset_recovery = 2.f;

          break;
      case eResonator:
          tau_recovery = 0.1f;
          coupling = 0.26f;
          reset_voltage = -65.f;
          reset_recovery = 2;

          break;
      default:
      case eRegular_spiking:
          tau_recovery = 0.02f;
          coupling = 0.2f;
          reset_voltage = -65.f;
          reset_recovery = 8;
          break;

    }
    neuronType = a_ntype;
  }

  /** Randomization as per Izhikevich 2003
  */
  void randomize(){

    float rand = noisefact * random(0, 1);
    switch(neuronType){ //<>//
      case eIntrinsically_bursting: //<>//
          tau_recovery = 0.02f;
          coupling = 0.2f;
          reset_voltage = -55; //<>//
          reset_recovery = 4; //<>//
          break; //<>//
      case eChattering:
          tau_recovery = 0.02f;
          coupling = 0.2f; //<>//
          reset_voltage = -50; //<>//
          reset_recovery = 2; //<>//
          break; //<>//
      case eFast_spiking:
          tau_recovery = 0.02 + 0.08*rand;
          coupling = 0.25f - 0.05*rand;
          //reset_voltage = -65;
          //reset_recovery = 2;

          break;
      case eLow_threshold:
          tau_recovery = 0.1f;
          coupling = 0.25f;
          reset_voltage = -65.f;
          reset_recovery = 2.f;

          break;
      case eResonator:
          tau_recovery = 0.1f;
          coupling = 0.26f;
          reset_voltage = -65.f;
          reset_recovery = 2;

          break;
      default:
      case eRegular_spiking:
          //tau_recovery = 0.02f;
          //coupling = 0.2f;
          reset_voltage = -65.f + 15.f*rand*rand;
          reset_recovery = 8.f - 6.f*rand*rand;
          break;

    }
  }
  
  void reset(){
    voltage = 0;
    recovery = 0;
  }
  
  void excite(float val){e_synapse.append(val);}
  void inhibit(float val){i_synapse.append(val);}
  
  
  
  void tick(){
    float[] ret = timeStep_Iz(  tau_recovery,
                  coupling,
                  reset_voltage,
                  reset_recovery,
                  e_synapse.array(),
                  i_synapse.array(),
                  dir_current,
                  voltage,
                  recovery);
     voltage = ret[0];
     recovery = ret[1];
     e_synapse.clear();
     i_synapse.clear();   //<>//
  }   //<>//
  
  // visualize
     //<>//
      //<>//
  float[] timeStep_Iz(             float a_a, // tau recovery   //<>//
                                float a_b, // coupling
                                float a_c, // reset voltage
                                float a_d, // reset recovery    //<>//
                                float[] e_syn, // synapse - contains excitation and inh scaled by synaptic vals, or zero if not spiked    //<>//
                                float[] i_syn,   //<>//
                                float a_i, // direct current   //<>//
                                float a_v, // in out excitation
                                float a_u  // recovery  
                                )   
  {   
    float fired = 0;  
    //for(int i=0; i < population_size; ++i) 
    if(a_v >= max_threshold)
            fired = 1.f;
            //numfired += 1;
    // numfired = sum(fired, population_size); // used for adenosine calc
    
    float v1 = a_v; 
    float u1 = a_u;
    float i1 = 0;
    //int synsize_x = excitation_size + inhibition_size;
    // float **syn_fired = create_matrix(synsize_x, population_size);

    //float *tmp = create_array(population_size);
    //sum(tmp, a_syn, synsize_x, population_size, 1); // sum along x axis

    //for(int j = 0; j < population_size; j++)
    //{
    if(fired == 1.f)
    {
        a_v = max_threshold;
        v1 = a_c;
        u1 = a_u + a_d;
    }
    float inputvlt = 0;
    for(int i=0; i<e_syn.length; i++){
      if(e_syn[i] >= max_threshold )
        inputvlt += e_syn[i];
    }
    for(int i=0; i<i_syn.length; i++){
      if(i_syn[i] >= max_threshold)
        inputvlt -= i_syn[i];
    }
        // sum up 
        // for(size_t i = 0; i < excitation_size + inhibition_size; i++)
        // {
        //     // TODO: change so inhibition is not removed
        //     if(a_syn[j] >= threshold)
        //         syn_fired[j] = a_syn[j];
        //     
        // }
    //}
    //float tmp = 0;//create_array(population_size);
    //sum(tmp, syn_fired, synsize_x, population_size, 1); // sum along x axis
    //sum(tmp, a_syn, synsize_x, population_size, 1); // sum along x axis
    //Stream<float> fs = Arrays.stream(a_syn); 
    //tmp = 
    i1 = inputvlt + a_i;
    //add(i1, tmp, a_i, population_size ); // add direct current
    
    // calculate output voltage
    float stepfact = 1.f/substeps;
    for(int step = 0; step < substeps; step++)
    {
      // v1=v1+(1.0/substeps)*(0.04*(v1**2)+(5*v1)+140-u + i1) 
      // 
      v1 += stepfact*(0.04*v1*v1 + 5*v1 + 
                  140-u1 + i1);
      // u1=u1+(1.0/substeps)*a*(b*v1-u1)               
    }
    u1 += (a_a*(a_b*v1 - u1));
    // multiply(v1, 100.f, population_size);
    // clip at threshold
    //for(int i = 0; i < population_size; i++)
    //{
    if(v1 >= max_threshold)
        v1 = max_threshold;
    //}
    if(debug)
    {
      println();
      println("-- " + name);
      println("inputvlt: " + inputvlt);
      printArray("exc syn_fired: ", e_syn);
      printArray("inh syn_fired: ", i_syn);
      println("vlt:" + voltage);
      println("i1: " + i1);
      
    }
    
    float[] retval = new float[2];
    retval[0] = v1;
    retval[1] = u1; 
    //copy_array(a_v, v1, population_size); // output voltage
    //copy_array(a_u, u1, population_size); // output recovery

    // destroy_matrix(syn_fired);
    return retval;
        
  }
  
  void debug(){
  println();
  println("-- " + name + " --");
  println ("tau: " + tau_recovery);
   println("coupling: " + coupling);
   println("reset_voltage: " + reset_voltage);
   println("reset_recovery: " + reset_recovery);
   println("dir_current: " + dir_current);
   println("voltage: " + voltage);
   println("recovery: " + recovery);
  print( "exc synapse: " + e_synapse.toString());
  print( "inh synapse: " + i_synapse.toString());
    
  }
}
