float[][] zeros(int rows, int cols){
  float[][] ret = new float[rows][cols];
  return ret;
}

float[] zeros (int dim){
  float [] ret = new float[dim];
  return ret;
}
float[][] id(int sz){
  float [][] ret = zeros(sz, sz);
  for (int j=0; j < ret.length; j++)
    for (int i=0; i < ret[0].length; i++)
      if (j==i) ret[j][i] = 1;
  return ret;
}

float[][] identity(int sz){
  return id(sz);
}

float[] ones (int sz){
  float[] ret = zeros(sz);
  for(int i=0; i<sz; i++)
    ret[i] = 1.f;
  return ret;
}

float[][] ones(int rows, int cols){
  return ones(rows, cols, 1.f);
}

float[][] ones(int rows, int cols, float val){
  float [][] ret = zeros(rows, cols);
  for (int j=0; j<ret.length; j++)
    for (int i=0; i<ret[0].length; i++)
      ret[j][i] = val;
  return ret;  
}

float[][] reset(float[][] a){
  for (int j=0; j<a.length; j++)
    for (int i=0; i<a[0].length; i++)
      a[j][i] = 0;
  return a;
}

float[][] multiply(float val, float[][] mat){
  float [][] ret = zeros(mat.length, mat[0].length);
  for (int j=0; j<ret.length; j++)
    for (int i=0; i<ret[0].length; i++)
      ret[j][i] = val * mat[j][i];
  return ret;
}

float[] multiply(float a, float[] b){
  float[] retval = new float[b.length];
  for(int j=0; j<b.length; j++)
    retval[j] = a*b[j];
  return retval;
}

float[] mult_per_elm(float[] a, float[] b){
  float[] retval = new float[a.length];
  for(int j=0; j<a.length; j++)
    retval[j] = a[j]*b[j];
  return retval;
}

float[][] addMatrix(float val, float[][] mat){
  float [][] ret = zeros(mat.length, mat[0].length);
  for (int j=0; j<ret.length; j++)
    for (int i=0; i<ret[0].length; i++)
      ret[j][i] = val + mat[j][i];
  return ret;
}

float[][] addMatrix(float[][] a, float[][] b){
  float [][] ret = zeros(a.length, a[0].length);
  for (int j=0; j<ret.length; j++)
    for (int i=0; i<ret[0].length; i++)
      ret[j][i] = a[j][i] + b[j][i];
  return ret;
}

float[] ravel(float[][] a){
  float []ret = zeros(a.length*a[0].length);
  int cnt = 0;
  for (int j=0; j<a.length; j++)
    for (int i=0; i<a[0].length; i++)
      ret[cnt++] = a[j][i];
  return ret;
}

float[] subtract(float[] a, float[] b){
  float [] ret = zeros(a.length);
  for (int j=0; j<ret.length; j++)
      ret[j] = a[j] - b[j];
  return ret;
}

float[] subtract(float[] a, float b){
  float[] r = zeros(a.length);
  for(int i=0; i<a.length; i++){
    r[i] = a[i] - b;
  }
  return r;
}

float[] divide(float[] a, float b){
  float [] ret = zeros(a.length);
  for (int j=0; j<ret.length; j++)
      ret[j] = a[j] / b;
  return ret;
}

float[][] subtractMatrix(float[][] a, float[][] b){
  float [][] ret = zeros(a.length, a[0].length);
  for (int j=0; j<ret.length; j++)
    for (int i=0; i<ret[0].length; i++)
      ret[j][i] = a[j][i] - b[j][i];
  return ret;
}



float[][] threshold(float val, float[][] mat){
  float [][] ret = zeros(mat.length, mat[0].length);
  for (int j=0; j<ret.length; j++)
    for (int i=0; i<ret[0].length; i++)
      if(mat[j][i] >= val)
        ret[j][i] = mat[j][i];
  return ret;
}

int argmax(float[] a){
  int retval = 0;
  
  for(int i=0; i<a.length; i++)
    if(a[retval] < a[i])
      retval = i;
  return retval;
}

float[] limitval(float lower, float upper, float[] a){
  float[] ret = zeros(a.length);
  for (int j=0; j<ret.length; j++){
    if(a[j] < lower) 
      ret[j] = lower;
    else if(a[j] > upper) 
      ret[j] = upper;
    else 
      ret[j] = a[j];
  }
  return ret;
  
}

float[][] limitval(float lower, float upper, float[][] a){
  float[][] ret = zeros(a.length, a[0].length);
  for (int j=0; j<ret.length; j++){
    for (int i=0; i<ret[0].length; i++){
      if(a[j][i] < lower) ret[j][i] = lower;
      else if(a[j][i] > upper) ret[j][i] = upper;
      else ret[j][i] = a[j][i];
    }
  }
  return ret;
  
}

float[][] getSubmatrix(int row, int col, int w, int h, float[][] m){
  float[][] ret = zeros(w, h);
  for (int j=0; j<ret.length; j++){
    for (int i=0; i<ret[0].length; i++){
      ret[j][i] = m[row+j][col+i];  
    }
  }

  return ret;
}

// mark - TAT additions
// void        set_submatrix(float *A, int ncols, float *S, int mrows, int mcols, int row, int col);
    //  Taken from: http://www.mymathlib.com/
    //  Arguments:                                                                //
    //     double *A    (Destination) Pointer to the first element of the matrix A[n][n].       //
    //     int    ncols The number of columns of the matrix A.                    //
    //     double *S    (Source) Source address of the submatrix.                     //
    //     int    mrows The number of rows matrix S.                              //
    //     int    mcols The number of columns of the matrix S.                    //
    //     int    row   (Offset) The row of A corresponding to the first row of S.         //
    //     int    col   (Offset) The column of A corresponding to the first column of S.   //
    //                                                                            //
float[][] setSubmatrix(float[][] A, float [][]S, 
  int row, int col)
{
    int ai,aj,si,sj;
    for ( aj = row, sj = 0; sj < S.length; aj++, sj++)
        for (si = 0, ai=col; si < S[0].length && ai<A[0].length; si++, ai++) 
          A[aj][ai] = S[sj][si];
    return A;
}

PVector depthToSensorCoords(float x, float y, float z,
  float xRes, float yRes, float fXToZ, float fYToZ)
{
    // compensate for perspective

    float tx = (float)((x / xRes - 0.5) * z * fXToZ);
    float ty = (float)((0.5 - y / yRes) * z * fYToZ);
    float tz = z;

    // shift to sensor coordinate system
    // y is pointing forwards and x to the side; z is up
    // PVector ret = new PVector(tz, -tx, -ty);
    PVector ret = new PVector(-tx, tz, -ty);
    // println("depthtosensor: " + ret.toString());
    //x = tz;
    //y = -tx;
    //z = -ty;
    return ret;
}

float [] h_rotation_matrix(int a, float alpha)
{
    float s = sin(alpha);
    float c = cos(alpha);
    float [] r = new float[16];
    if(a== 0){
            r[ 0] = 1; r[ 1] = 0; r[ 2] = 0; r[ 3] = 0;
            r[ 4] = 0; r[ 5] = c; r[ 6] =-s; r[ 7] = 0;
            r[ 8] = 0; r[ 9] = s; r[10] = c; r[11] = 0;
            r[12] = 0; r[13] = 0; r[14] = 0; r[15] = 1;
    }
    else if(a== 1){
            r[ 0] = c; r[ 1] = 0; r[ 2] = s; r[ 3] = 0;
            r[ 4] = 0; r[ 5] = 1; r[ 6] = 0; r[ 7] = 0;
            r[ 8] =-s; r[ 9] = 0; r[10] = c; r[11] = 0;
            r[12] = 0; r[13] = 0; r[14] = 0; r[15] = 1;
    }
    else if(a==2){
            r[ 0] = c; r[ 1] = -s; r[ 2] = 0; r[ 3] = 0;
            r[ 4] = s; r[ 5] = c; r[ 6] = 0; r[ 7] = 0;
            r[ 8] = 0; r[ 9] = 0; r[10] = 1; r[11] = 0;
            r[12] = 0; r[13] = 0; r[14] = 0; r[15] = 1;
            
    }
    return r;
}

float [] h_translation_matrix(float tx, float ty, float tz)
{
    float[] r = zeros(16);
    r[ 0] = 1; r[ 1] = 0; r[ 2] = 0; r[ 3] = tx;
    r[ 4] = 0; r[ 5] = 1; r[ 6] = 0; r[ 7] = ty;
    r[ 8] = 0; r[ 9] = 0; r[10] = 1; r[11] = tz;
    r[12] = 0; r[13] = 0; r[14] = 0; r[15] = 1;
    return r;
}

float[] h_perspective_matrix(float far, float near, float fov){
    float[] r = zeros(16);
    float a = -far/(far-near);
    float b = -far*near/(far-near);
    float S = 1/(tan(fov*0.5f*PI/180.f));
    r[ 0] = S; r[ 1] = 0; r[ 2] = 0; r[ 3] = 0;
    r[ 4] = 0; r[ 5] = S; r[ 6] = 0; r[ 7] = 0;
    r[ 8] = 0; r[ 9] = 0; r[10] = a; r[11] = -1;
    r[12] = 0; r[13] = 0; r[14] = b; r[15] = 1;
    return r;
}
    
float [] h_scaling_matrix(float sx, float sy, float sz)
{
    float[] r = new float[16];
    r[ 0] =sx; r[ 1] = 0; r[ 2] = 0; r[ 3] = 0;
    r[ 4] = 0; r[ 5] =sy; r[ 6] = 0; r[ 7] = 0;
    r[ 8] = 0; r[ 9] = 0; r[10] =sz; r[11] = 0;
    r[12] = 0; r[13] = 0; r[14] = 0; r[15] = 1;
    return r;
}

float [] h_multiply_v(float[] m, float[] v)
{
    float[] r = new float [4];
    float[] t = new float [4];
    t[0] = m[ 0]*v[0] + m[ 1]*v[1] + m[ 2]*v[2] + m[ 3]*v[3];
    t[1] = m[ 4]*v[0] + m[ 5]*v[1] + m[ 6]*v[2] + m[ 7]*v[3];
    t[2] = m[ 8]*v[0] + m[ 9]*v[1] + m[10]*v[2] + m[11]*v[3];
    t[3] = m[12]*v[0] + m[13]*v[1] + m[14]*v[2] + m[15]*v[3];
    //if(t[3] != 0 && false) // normalize vector
    //{
    //  println("a");
    //    r[0] = t[0] / t[3];
    //    r[1] = t[1] / t[3];
    //    r[2] = t[2] / t[3];
    //    r[3] = 1;
    //}else {   
    //  println("b");
        r[0] = t[0];
    r[1] = t[1];
    r[2] = t[2];
    r[3] = 1;
    //}

    return r;
}

float[][] rotateMatrix(float[][] m, float rot){
  return rotateMatrix(m, rot, m[0].length/2.f, m.length/2.f,
    m[0].length, m.length); 
}

float[][] rotateMatrix(float[][] m, 
  float rot, 
  float origo_x, 
  float origo_y,
  int out_x,
  int out_y){
    
  float[][] input_matrix = m;
  int output_matrix_size_y = out_y;
  int output_matrix_size_x = out_x; 
  
  int input_matrix_size_y = m.length;
  int input_matrix_size_x = m[0].length; 
  float[][] output_matrix = zeros(out_y, out_x);
  float[] element = new float[4];
   int src_i=0;
    int src_j=0;
    // update rotation matrix
    float [] rot_mat = h_rotation_matrix(2, rot);
    float[] trans_mat = h_translation_matrix(
                           -origo_x, //input_matrix_size_x/2.0,
                           -origo_y, //input_matrix_size_y/2.0,
                            0);
    float[] detrans_mat = h_translation_matrix(                          
                           origo_x, //input_matrix_size_x/2.0,
                           origo_y, //input_matrix_size_y/2.0,
                           0);
    //reset_matrix(output_matrix, input_matrix_size_x, input_matrix_size_y);
    // iterate over target matrix
    for (int i=0; i<output_matrix_size_y; i++) {
        for (int j=0; j<output_matrix_size_x; j++) {

            element[0] = (float)j;
            element[1] = (float)i;
            element[3] = 1.f;
            // detransl * rot * transl * element
            float[] tmp1 = h_multiply_v(trans_mat, element);
            float[] tmp2 = h_multiply_v( rot_mat, tmp1);
            tmp1 = h_multiply_v(detrans_mat, tmp2);
            src_i = Math.round(tmp1[1]);
            src_j = Math.round(tmp1[0]);
            if(src_i>=0 && src_i<input_matrix_size_y &&
               src_j>=0 && src_j<input_matrix_size_x)
                output_matrix[i][j] = input_matrix[src_i][src_j];
            //if(debugmode)
            //{
            //    printf("src_i=%i, src_j=%i - ", src_i, src_j);
            //    printf("output at %i, %i = %f\n", i, j, output_matrix[i][j]);
            //}
        }
    }
    return output_matrix;
}

float[][] deprojectMatrix(float[][] m, float near, float far, float fov){
  int output_matrix_size_y = m.length;
  int output_matrix_size_x = m[0].length;
  int input_matrix_size_y = m.length;
  int input_matrix_size_x = m[0].length; 
  float[] element = new float[4];
  
  // get x and y translations
    int src_i = 0;
    int src_j = 0;
    float[] projmat = h_perspective_matrix(near, far, fov);
    float[][] output_matrix = zeros(output_matrix_size_y, output_matrix_size_x);
    for (int i=0; i<output_matrix_size_y; i++) {
        for (int j=0; j<output_matrix_size_x; j++) {
            element[0] = (float)j;
            element[1] = (float)i;
            element[3] = 1.f;
            float[] tmp = h_multiply_v(projmat, element);
            src_i = Math.round(tmp[1]);
            src_j = Math.round(tmp[0]);
            if(src_i>=0 && src_i<input_matrix_size_y &&
               src_j>=0 && src_j<input_matrix_size_x){
                output_matrix[i][j] = m[src_i][src_j];
                // printf("input assigned: %f", input_matrix[src_i][src_j]);
            }

        }
    }

    
  return output_matrix;
}

float[][] scaleMatrix(float[][] m, float sx, float sy, int out_x, int out_y){
  int output_matrix_size_y = out_y;
  int output_matrix_size_x = out_x;
   int input_matrix_size_y = m.length;
  int input_matrix_size_x = m[0].length; 
  float[] element = new float[4];
  float[] trans_mat = h_translation_matrix(
                           -input_matrix_size_x/2.0,
                           -input_matrix_size_y/2.0,
                            0);
    float[] detrans_mat = h_translation_matrix(                          
                           input_matrix_size_x/2.0,
                           input_matrix_size_y/2.0,
                           0);
   // get x and y Scales
    int src_i = 0;
    int src_j = 0;
    float [] scale_mat = h_scaling_matrix(1.f/sx, 1.f/sy, 0);
    float [][] output_matrix = zeros(out_y, out_x);
    for (int i=0; i<output_matrix_size_y; i++) {
        for (int j=0; j<output_matrix_size_x; j++) {
            element[0] = (float)j;
            element[1] = (float)i;
            element[3] = 1.f;
            float[] tmp1 = h_multiply_v(trans_mat, element);
            float[] tmp2 = h_multiply_v(scale_mat, tmp1);
            tmp1 = h_multiply_v(detrans_mat, tmp2);
            src_i = Math.round(tmp1[1]);
            src_j = Math.round(tmp1[0]);
            if(src_i>=0 && src_i<input_matrix_size_y &&
               src_j>=0 && src_j<input_matrix_size_x){
                output_matrix[i][j] = m[src_i][src_j];
                // printf("input assigned: %f", input_matrix[src_i][src_j]);
            }
            //if(debugmode)
            //{
            //    printf("dst_i=%i, dst_j=%i, src_i=%i, src_j=%i - ",i, j, src_i, src_j);
            //    printf("output at %i, %i = %f\n", i, j, output_matrix[i][j]);
            //}
        }
    }

    //if(debugmode)
    //{
    //    // print out debug info
    //    printf("scale: %f, %f\n", x_scale[0], y_scale[0]);
    //}
    return output_matrix;
}

float[][] translateMatrix(float[][]m, float x, float y, int out_x, int out_y)
{
   int output_matrix_size_y = out_y;
  int output_matrix_size_x = out_x;
    int input_matrix_size_y = m.length;
  int input_matrix_size_x = m[0].length; 
  float[] element = new float[4];
  float[] trans_mat = h_translation_matrix(
                           -input_matrix_size_x/2.0,
                           -input_matrix_size_y/2.0,
                            0);
  // get x and y translations
    int src_i = 0;
    int src_j = 0;
    trans_mat = h_translation_matrix( -x, -y, 0);
    float[][] output_matrix = zeros(out_y, out_x);
    for (int i=0; i<output_matrix_size_y; i++) {
        for (int j=0; j<output_matrix_size_x; j++) {
            element[0] = (float)j;
            element[1] = (float)i;
            element[3] = 1.f;
            float[] tmp = h_multiply_v(trans_mat, element);
            src_i = Math.round(tmp[1]);
            src_j = Math.round(tmp[0]);
            if(src_i>=0 && src_i<input_matrix_size_y &&
               src_j>=0 && src_j<input_matrix_size_x){
                output_matrix[i][j] = m[src_i][src_j];
                // printf("input assigned: %f", input_matrix[src_i][src_j]);
            }

        }
    }


  return output_matrix;
}

boolean similar(float a, float b, float delta){
  return abs(a-b) < delta;
}

float signedRelAngle(PVector a, PVector b){

  float a2 = a.heading();
  float a1 = b.heading();
  float sign = a1 > a2 ? 1 : -1;
  float angle = a1 - a2;
  float K = -sign * PI * 2;
  angle = (abs(K+angle) < abs(angle)) ? K + angle : angle;
  return angle;

}

float sumMatrix(float[][] a){
  float r = 0;
  r = sumArray(sumHor(a));
  return r;
}

float[] sumHor(float[][] a){
  float[] r = zeros(a.length);
  for(int i=0; i<a.length; i++){
    r[i] = sumArray(a[i]);
  }
  return r;
}

float sumArray(float[] a){
  float r = 0;
  for(int i=0; i<a.length; i++){
    r += a[i];
  }
  return r;
}

float[] range_expand(float low, float high, float min, float max, float[] a){
  float[] r = zeros(a.length);
  for(int i=0; i<a.length; i++){
    if (a[i] < low) r[i] = min;
    else if(a[i] > high) r[i] = max;
    else r[i] = a[i];
  }
  return r;
}
  
float[] map(float[] val, float src_lo, float src_hi, float tgt_lo, float tgt_hi){
  float[] ret = zeros(val.length);
  for(int i=0; i<ret.length; i++){
    ret[i] = map(val[i], src_lo, src_hi, tgt_lo, tgt_hi);
  }
  return ret;
}

float[][] map(float[][] val, float src_lo, float src_hi, float tgt_lo, float tgt_hi){
  float[][] ret = zeros(val.length, val[0].length);
  for(int i=0; i<ret.length; i++){
    ret[i] = map(val[i], src_lo, src_hi, tgt_lo, tgt_hi);
  }
  return ret;
}

float[] numSeries(int hinum){
  float[] ret = zeros(hinum);
  for(int i=0; i<hinum; i++){
    ret[i] = (float) i;
  }
  return ret;
}
