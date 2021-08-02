
class LeakyIntegrator{
 
  float[][] store;
  float[][] input;
  float growth;
  float accumulate;
  float decaythreshold;
  float decayfactor;
  
  LeakyIntegrator(
    int rows, int cols, 
    float agrowth, float adecay, 
    float aaccumulate, float adecaythreshold){
   growth = agrowth;
   decayfactor = adecay;
   accumulate = aaccumulate;
   decaythreshold = adecaythreshold;
   store = new float[rows][cols];
  }
  
  void tick(){
    for(int j=0; j<store.length; j++)
      for(int i=0; i<store[0].length; i++){
        float epsilon = growth;
        float lambda = accumulate;
        if(input[j][i] < decaythreshold){
          epsilon = decayfactor;
          lambda = 0;
        }
        store[j][i] += epsilon*(input[j][i] - (1-lambda)*store[j][i]);
        
      }
  }
  
  void setInput(float[][] inp){
    input = inp; 
  }
  
  void setGrowth(float growth){
    this.growth = growth;
  }
  
  void setAccumulate(float acc){
    this.accumulate = acc;
  }
  
  void setDecayThreshold(float thr){
    this.decaythreshold = thr;
  }
  
  void setDecayFactor(float fact){
    this.decayfactor = fact;
  }
  
  void reset(){
    store = new float[store.length][store[0].length];
  }
  
  float[][] getOutput(){
   return store; 
  }
}

class LimitedLeakyIntegrator extends LeakyIntegrator{
  float limit;
  
  LimitedLeakyIntegrator(
    int rows, int cols, 
    float agrowth, float adecay, 
    float aaccumulate, float adecaythreshold, float alimit){
   
    super(rows, cols, agrowth, adecay, aaccumulate, adecaythreshold);
    limit = alimit;
  }
  
  void tick(){
    for(int j=0; j<store.length; j++)
      for(int i=0; i<store[0].length; i++){
        float epsilon = growth;
        float lambda = accumulate;
        if(input[j][i] < decaythreshold){
          epsilon = decayfactor;
          lambda = 0;
        }
        store[j][i] += epsilon*(input[j][i] - (1-lambda)*store[j][i]);
        store[j][i] = min(store[j][i], limit);
        
      }
  }
}
