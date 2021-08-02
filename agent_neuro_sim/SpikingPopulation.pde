/*
Spiking population with connection topology
*/

class SpikingPopulation{
 
  SpikingUnit[] units;
  Buffer[] data;
  float [] output;
  float [][] internal_synapse;
  String name;
  // constructor
  SpikingPopulation(String aname, int sz, NeuronType ntype, int bufsize){
    units = new SpikingUnit[sz];
    output = new float[sz];
    data = new Buffer[sz];
    for(int i=0; i<sz; i++){
      units[i] = new SpikingUnit(aname+"_"+(i+1), ntype, 2);
      data[i] = new Buffer(bufsize);
    }
    // default internal synapse is no conn
    internal_synapse = zeros(sz, sz);
    name = aname;
  }

  SpikingPopulation(String aname, int sz, NeuronType ntype, int bufsize, float noise){
    units = new SpikingUnit[sz];
    output = new float[sz];
    data = new Buffer[sz];
    for(int i=0; i<sz; i++){
      units[i] = new SpikingUnit(aname+"_"+(i+1), ntype, 2, noise);
      data[i] = new Buffer(bufsize);
    }
    // default internal synapse is no conn
    internal_synapse = zeros(sz, sz);
    name = aname;
    
  }
  
  // accessors
  String getName(){ return name;}
  int getSize(){return units.length;}
  
  float[] getOutput(){
    for(int i=0; i<units.length; i++)
      output[i] = units[i].getOutput();
    return output;
  }

  float[] getNormOutput(){
    for(int i=0; i<units.length; i++)
      output[i] = units[i].getNormOutput();
    return output;
  }
  
  Buffer[] getBuffers(){
    return data;  
  }
  
  void reset(){
    for(int i=0; i<units.length; i++){
      units[i].reset();
    }
  }

  
  void setInternalTopology(float[][] top){
    internal_synapse = top;  
  }
    
  
  // methods
  void tick(){
    for(int i=0; i<units.length; i++){
      units[i].tick();
      data[i].append(units[i].getOutput());
    }
    updateConn(internal_synapse, units);
    
  }
  
  void setDirect(float[] val){
    for(int i=0; i<units.length; i++)
      units[i].setDirect(val[i]);
  }
  
  void setResetVoltage(float val){
    // will increase, decrease all 
    for(int i=0; i<units.length; i++)
      units[i].setResetVoltage(val);
  }
  
  void excite(float[] val, float[][] synapse){
    // will throw array out of bounds if not matching
    for(int j=0; j<synapse.length; ++j){
      for(int i=0; i<synapse[0].length; ++i){
          if(synapse[j][i] > 0)
            units[i].excite(synapse[j][i] * val[j]);
      }
    }
    
  }
  
  void inhibit(float[] val, float[][] synapse){
    /**
    val - array of unit outputs (any size)
    synapse - matrix of synapse mappings where 
      rows=sizeof(val), cols=sizeof(this population)
    */
    // will throw array out of bounds if not matching
    for(int j=0; j<synapse.length; ++j){
      for(int i=0; i<synapse[0].length; ++i){
          if(synapse[j][i] > 0)
            units[i].inhibit(synapse[j][i] * val[j]);
      }
    }
  }
  
  void modulateResetVoltage(float diff){
    // will increase, decrease all 
    for(int i=0; i<units.length; i++)
      units[i].modulateResetVoltage(diff);
  }
  
  void modulateResetVoltage(float[] diff){
    // will increase, decrease per unit 
    for(int i=0; i<units.length; i++)
      units[i].modulateResetVoltage(diff[i]);
  }

  void randomize(){
    for(int i=0; i<units.length; i++)
      units[i].randomize();
  }
  
  // privates
  private void updateConn(float[][] matrix, SpikingUnit[] units){

    for(int j=0; j<matrix.length; ++j){
      for(int i=0; i<matrix[0].length; ++i){
        if(matrix[j][i] > 0)
          units[i].excite(units[j].getOutput());//, units[j].getPhase());
        else if(matrix[j][i] < 0)
          units[i].inhibit(units[j].getOutput());//, units[j].getPhase());
      }
    }
  }
}
