
float[][] zCross(float[][] rowVec) {
  return new float[][]{{-rowVec[1][0]},{rowVec[0][0]},{0}};
}

float[][] Cross(float[][] u, float[][] v) {
  return new float[][]{{u[1][0]*v[2][0]-u[2][0]*v[1][0]},
                       {u[2][0]*v[0][0]-u[0][0]*v[2][0]},
                       {u[0][0]*v[1][0]-u[1][0]*v[0][0]}};
}

float[][] getXYZrotation(float x, float y, float z) {
  float sx,cx,sy,cy,sz,cz;
  sx = sin(radians(x));
  cx = cos(radians(x));
  sy = sin(radians(y));
  cy = cos(radians(y));
  sz = sin(radians(z));
  cz = cos(radians(z));
  float[][] xrot = {{1,  0, 0 },
                    {0, cx,sx},
                    {0, -sx, cx}};
  float[][] yrot = {{cy,  0,-sy },
                    {0 , 1, 0},
                    {sy, 0, cy}};
  float[][] zrot = {{cz,sz, 0},
                    {-sz, cz, 0},
                    {0 ,  0, 1}};
  return Mat.multiply(Mat.multiply(yrot,xrot),zrot);
}

class System {
  // Fields
  private int _last_frame, _frame_time;
  
  // Contructors
  System() {
    _last_frame = millis();
    _frame_time = 1;
  }
  
  // Properties
  int frame_ms() { return _frame_time; }
  float frame_sec() { return (float)_frame_time/1000.0; }
  
  // Methods
  void update() {
    _frame_time = millis()-_last_frame;
    _last_frame = millis();
    if (_frame_time==0) {
      _frame_time = 1;
    }
  }
}

class TransformMatrix {
  // Fields
  private float[][] _H;
  private Quaternion _Q;
  
  //Constructors
  TransformMatrix() {
    _H = new float[4][4];
                    
    _Q = new Quaternion();
  }
  TransformMatrix(float[][] h) {
    _H = h;
    _Q = new Quaternion();
    _Q.fromRotationMatrix(rotation());
  }
  TransformMatrix(float h11,float h12,float h13,float h14,float h21,float h22,float h23,float h24,float h31,float h32,float h33,float h34) {
    _H = new float[][]{{h11,h12,h13,h14},
                      {h21,h22,h23,h24},
                      {h31,h32,h33,h34},
                      { 0,  0,  0 , 1 }};
    _Q = new Quaternion();
    _Q.fromRotationMatrix(rotation());
  }
  
  // Properties
    //Setters for H and Q
  void setH(float h11,float h12,float h13,float h14,float h21,float h22,float h23,float h24,float h31,float h32,float h33,float h34) {
    _H = new float[][]{{h11,h12,h13,h14},
                      {h21,h22,h23,h24},
                      {h31,h32,h33,h34},
                      { 0,  0,  0 , 1 }};
    _Q.fromRotationMatrix(rotation());
  }
  void setQ(Quaternion newQ) {rotation(newQ);}
  
      //Setters for positions of transform matrix
  void pos(float[] newPos) { 
    for (int i=0;i<newPos.length;i++){
      _H[i][3] = newPos[i];
    }
  }
  void pos(float x, float y, float z) {
    _H[0][3] = x;
    _H[1][3] = y;
    _H[2][3] = z;
  }
  void pos(PVector newPos) {
    _H[0][3] = newPos.x;
    _H[1][3] = newPos.y;
    _H[2][3] = newPos.z;
  }
    //Setters for the rotation component of transform matrix
  void rotation(float h11,float h12,float h13,float h21,float h22,float h23,float h31,float h32,float h33) {
    _H[0][0] = h11;
    _H[0][1] = h12;
    _H[0][2] = h13;
    _H[1][0] = h21;
    _H[1][1] = h22;
    _H[1][2] = h23;
    _H[2][0] = h31;
    _H[2][1] = h32;
    _H[2][2] = h33;
    _Q.fromRotationMatrix(Mat.subMatrix(_H,0,2,0,2));
  }
  void rotation(float[][] newRotation) {
    for (int i=0;i<3;i++) {
      for (int j=0;j<3;j++) {
        _H[i][j] = newRotation[i][j];
      }
    }
    _Q.fromRotationMatrix(Mat.subMatrix(_H,0,2,0,2));
  }
  void rotation(Quaternion newRotation) {
    _Q = new Quaternion(newRotation);
    float[][] newRot = _Q.rotationMatrix();
    for (int i=0;i<3;i++) {
      for (int j=0;j<3;j++) {
        _H[i][j] = newRot[i][j];
      }
    }
  }
  
    // Getters for H and Q
  float[][] H() {return _H;}   
  Quaternion Q() { return _Q; } 
  
    //getters for rotation and positions
  float[][] rotation() { return Mat.subMatrix(_H,0,2,0,2); } 
  PVector posVec() { return new PVector(_H[0][3],_H[1][3],_H[2][3]); }  
  float[] pos() { return shorten(Mat.column(_H,3)); }
  float[][] posCol() { return Mat.subMatrix(_H,0,2,3,3); }
  float[][] posRow() { return Mat.transpose(Mat.subMatrix(_H,0,2,3,3)); }
  
  // Methods
  void printLocation(int xpos, int ypos) {
    textFont(gui.display(), 15);
    text( "End Effector Tip:"+
          "\nX: "+ String.format("%.2f",_H[0][3])+
          "\nY: "+ String.format("%.2f",_H[1][3])+
          "\nZ: "+ String.format("%.2f",_H[2][3]),xpos,ypos);
  }
  
  float[][] adjoint(float[][] matrix) {
    int len = matrix[0].length;
    float[][] cofactor = new float[len][len];
    int sign = 1;
    for (int i=0;i<len;i++) {
      for (int j=0;j<len;j++) {
        cofactor[i][j] = sign*subDet(matrix,i,j,len);
        sign *= -1;
      }
    }
    return Mat.transpose(cofactor);
  }
  
  float subDet(float[][] matrix, int row, int col, int len) {
    int m, n;
    if (row==0) {m=1;} 
    else        {m=0;}
    if (col==0) {n=1;}
    else        {n=0;}
    float[][] subMatrix = new float[len-1][len-1];
    for (int i=0;i<len;i++) {
      for (int j=0;j<len;j++) {
        if (i!=row && j!=col) {
          subMatrix[m][n] = _H[i][j];
          n++;
        }
      }
      m++;
    }
    return Mat.det(subMatrix);
  }
    
}
