// Tragectory class, Used to make and keep track of tragectories. Pass start, final position and velocity in row format.
class Trajectory {
  // Fields
  private float[][][] _quinticCurves;              // 6 Coefficients for trajectory in end effector space [parameter, x y z alpha][points][6 vars]
  private float[] _trajectoryTime;                 // list of times to get to each point including origin
  private int _perameters;                         // Number of parameters, will be 4 in end effector space, x,y,z,alpha
  private float[][] _xi,_vi;                       // Matrix of position and velocity that match with _trajectoryTime, including origin [parameters, x y z alpha][points]
  private PVector[] _rotationAxis;                 // List of rotation axis accociated with each point translation (in origin)
  private boolean _isMotorSpace;
  
  
// Constructors

  // Takes in a list of poses as TransformMatrix and time at each point begining with current pose and pathTime[0]=0
  Trajectory(TransformMatrix[] poses, float[] pathTime) {
    _perameters = 4;    
    _xi = new float[_perameters][poses.length];
    _vi = new float[_perameters][poses.length];
    _quinticCurves = new float[_perameters][pathTime.length-1][6];
    _rotationAxis = new PVector[poses.length-1];
    _trajectoryTime = pathTime;
    _isMotorSpace = false;
    
    getXiVi(poses, _trajectoryTime);                                // This populates _xi, _vi, _rotationAxis with values
    populateCurves(pathTime, _xi, _vi);                             // This populates _quinticCurves with values
  }
  
  // Creates a trajectory of multipul parameters but one point
  Trajectory(float pathTime, float[] startPosition, float[] finalPosition) {
    _perameters = startPosition.length;   
    _xi = new float[_perameters][2];
    _vi = new float[_perameters][2];
    _quinticCurves = new float[_perameters][2][6];
    _trajectoryTime = new float[]{0,pathTime};
    _isMotorSpace = true;
    
    // This populates _xi, _vi, _rotationAxis with values
    for (int i=0;i<_perameters;i++) {
      _xi[i][0] =  radians(startPosition[i]);   
      _xi[i][1] =  finalPosition[i]; 
      _vi[i] = new float[]{0,0};
    }      
    // Populates _quinticCurves, cant use populateCurves() as that assumes xyz alpha
    for (int i=0;i<_perameters;i++) {
      _quinticCurves[i] = getCurve(_trajectoryTime, _xi[i], _vi[i]);
    }
  }
  /*
  // Takes in a list of points where each COLUMN of position and velocity is a new point.
  Trajectory(int perameters, float[] pathTime, float[][] Position, float[][] Velocity) {
    _perameters = perameters;
    _quinticCurves = new float[pathTime.length][_perameters][6];
    _trajectoryTime = pathTime;
    
    for (int i=0;i<_perameters;i++) {
      _quinticCurves[i] = getCurve(pathTime, Position[i], Velocity[i]);
    }
    
  }
  */
  
// Properties
  float[][] xcoeffs() {return _quinticCurves[0];}
  float[][] ycoeffs() {return _quinticCurves[1];}
  float[][] zcoeffs() {return _quinticCurves[2];}
  float[][] alphacoeffs() {return _quinticCurves[3];}
  float[][] coeffs(float time) { return _quinticCurves[checkTime(time)]; }
  boolean isMotorSpace() { return _isMotorSpace; }
  
// Methods  
  // This function populates _xi and _vi denpending on the number of poses.
  // Also populates the rotationAxis in orgin coordinates
  void getXiVi(TransformMatrix[] poses, float[] time) {
    float[][][] R12;
    float[][] axisOrigin = new float[3][1];
    Quaternion Q12;
  
    Q12 = new Quaternion();
    R12 = new float[poses.length-1][3][3];
    
  // Defines Xi for x,y,z as rotation must be calculated from R12
    for (int j=0;j<3;j++){
      for (int i=0;i<poses.length;i++) {
        _xi[j][i] = poses[i].pos()[j];
      }
    }
  // Setup start rotation as zero
    _xi[3][0] = 0;
    
  // For each pose find the rotation axis and final angle 
  // where it calculates the i+1 location, ie i=0 will find _xi[1]
    for (int i=0;i<poses.length;i++) {
    // If it is NOT at the end
      if (i!=poses.length-1) {
      R12[i] = Mat.multiply(Mat.transpose(poses[i].rotation()),poses[i+1].rotation());  
      Q12.fromRotationMatrix(R12[i]);
    // Set the rotation angles and axis
      _xi[3][i+1] = Q12.angle();
      axisOrigin = Mat.multiply(poses[i+1].rotation(),new float[][]{{Q12.axis().x},{Q12.axis().y},{Q12.axis().z}});
      _rotationAxis[i] = new PVector(axisOrigin[0][0],axisOrigin[1][0],axisOrigin[2][0]);
      }
    // Calculate the velocity lists
    // If it is at the end or start set velocity to zero
      if (i==poses.length-1 || i==0) {
        for (int j=0;j<3;j++){
          _vi[j][i] = 0;
        }
    // Else (not start or finish) Then set the velocity to be average or zero if change direction
      } else {
        for (int j=0;j<3;j++){
          if ((_xi[j][i+1]-_xi[j][i])/abs(_xi[j][i+1]-_xi[j][i])!=(_xi[j][i]-_xi[j][i-1])/abs(_xi[j][i]-_xi[j][i-1])) {
            _vi[j][i] = 0;
          } else {
            _vi[j][i] = (_xi[j][i+1]-_xi[j][i-1])/2;
          }
        }
      }
      // Rotation velocity is zero
      _vi[3][i] = 0;
    }
  }
  
  // Specific to Pose command as it handels the 4th parameter differently
  void populateCurves(float[] time,float[][] xi,float[][] vi) {
    // Populates the x,y,z parameters
    for (int i=0;i<3;i++) {
      _quinticCurves[i] = getCurve(time, xi[i], vi[i]);
    }
    // Populates the rotation parameter 
    for (int i=0;i<time.length-1;i++) {
      if (time[i]!=time[i+1]) {
        _quinticCurves[3][i] = getCurve(time[i],time[i+1],0,xi[3][i+1],0,0);
      }
      else _quinticCurves[3][i] = new float[]{0,0,0,0,0,0};
    }
  }
  
// Creates coefficient vectors for different positions, 
// conserves velocity and acceleration so only good for xyz
  float[][] getCurve(float[] tf,float[] xi, float[] vi) {
    float[][] coeffs = new float[tf.length-1][6];
    coeffs[0] = getCurve(tf[0],tf[1],xi[0],xi[1],vi[0],vi[1]);
    for (int i=1;i<tf.length-1;i++) {
      if (tf[i]!=tf[i-1]) {
        coeffs[i] = getCurve(tf[i],tf[i+1],xi[i],xi[i+1],vi[i],vi[i+1],getAcceleration(tf[i],coeffs[i]));
      }
      else coeffs[i] = new float[]{xi[i],vi[i],0,0,0,0};
    }
    return coeffs;
  }
  
  
  float[] getCurve(float t0, float tf,float x0, float xf,float v0, float vf) {
    if (tf!=t0) {
      float[][] Y, A;
      double[][] X;
      float tm =t0 + (tf-t0)/2;
      A  = new float[][] {{1,  t0  ,pow(t0,2),pow(t0,3)  , pow(t0,4)  , pow(t0,5)  },  // start position
                          {0,  1   , 2*t0    ,3*pow(t0,2), 4*pow(t0,3), 5*pow(t0,4)},  // start velocity 
                          {1,  tm  ,pow(tm,2),pow(tm,3)  ,  pow(tm,4) , pow(tm,5)  },  // mid position is halfway
                          {1,  tf  ,pow(tf,2),pow(tf,3)  ,  pow(tf,4) , pow(tf,5)  },  // final position
                          {0,  1   , 2*tf    ,3*pow(tf,2), 4*pow(tf,3), 5*pow(tf,4)},  // final velocity
                          {0,  1   , 2*tm    ,3*pow(tm,2), 4*pow(tm,3), 5*pow(tm,4)}}; // mid velocity is average velocity
      Y  = new float[][] {{x0},
                          {v0},
                          {x0+(xf-x0)/2},
                          {xf},
                          {vf},
                          {(xf-x0)/(0.6*(tf-t0))}};
      X = new QR(A).solve(Y);
      return new float[] {(float)X[0][0],(float)X[1][0],(float)X[2][0],(float)X[3][0],(float)X[4][0],(float)X[5][0]};
    }
    else return new float[] {x0,0,0,0,0,0};
  }
  
  float[] getCurve(float t0, float tf,float x0, float xf,float v0, float vf, float a0) {
    if (tf!=t0) {
      float[][] Y, A;
      double[][] X;
      float tm =t0 + (tf-t0)/2;
      A  = new float[][] {{1,  t0  ,pow(t0,2),pow(t0,3)  , pow(t0,4)  , pow(t0,5)  },  // start position 
                          {0,  1   , 2*t0    ,3*pow(t0,2), 4*pow(t0,3), 5*pow(t0,4)},  // start velocity
                          {1,  tm  ,pow(tm,2),pow(tm,3)  ,  pow(tm,4) , pow(tm,5)  },  // mid position
                          {1,  tf  ,pow(tf,2),pow(tf,3)  ,  pow(tf,4) , pow(tf,5)  },  // final position
                          {0,  1   , 2*tf    ,3*pow(tf,2), 4*pow(tf,3), 5*pow(tf,4)},  // final velocity
                          {0,  0   , 2       ,    6*t0   ,12*pow(t0,2),20*pow(t0,3)}}; // start acceleration
      Y  = new float[][] {{x0},
                          {v0},
                          {x0+(xf-x0)/2},
                          {xf},
                          {vf},
                          {a0}};
      X = new QR(A).solve(Y);
      return new float[] {(float)X[0][0],(float)X[1][0],(float)X[2][0],(float)X[3][0],(float)X[4][0],(float)X[5][0]};
    }
    else return new float[] {x0,0,0,0,0,0};
  }
  
  
  
  
  // Position and velocity and acceleration of the parameters, x,y,z,alpha for end effector
  // Used to generate the end effector X dot to multiply with Jacobian
  float[] getPosition(float t) {
    float[][] a = new float[_perameters][6];
    float[] pos = new float[_perameters];
    for (int i=0;i<_perameters;i++) {
      a = _quinticCurves[i];
      int j = checkTime(t);
      pos[i]= a[j][0]+a[j][1]*pow(t,1)+a[j][2]*pow(t,2)+a[j][3]*pow(t,3)+a[j][4]*pow(t,4)+a[j][5]*pow(t,5);
    }
    return pos;
  }
  
  float[] getVelocity(float t) {
    float[][] a = new float[_trajectoryTime.length][6];
    float[] vel = new float[_perameters];
    for (int i=0;i<_perameters;i++) {
      a = _quinticCurves[i];
      int j = checkTime(t);
      vel[i]= a[j][1]+2*a[j][2]*pow(t,1)+3*a[j][3]*pow(t,2)+4*a[j][4]*pow(t,3)+5*a[j][5]*pow(t,4);
    }
    return vel;
  }
  
  float[] getAcceleration(float t) {
    float[][] a = new float[_perameters][6];
    float[] accel = new float[_perameters];
    for (int i=0;i<_perameters;i++) {
      a = _quinticCurves[i];
      int j = checkTime(t);
      accel[i]= 2*a[j][2]+6*a[j][3]*pow(t,1)+12*a[j][4]*pow(t,2)+20*a[j][5]*pow(t,3);
    }
    return accel;
  }
  float getAcceleration(float t, float[] a) {
    return 2*a[2]+6*a[3]*pow(t,1)+12*a[4]*pow(t,2)+20*a[5]*pow(t,3);
  }
  
  // Used when there are multiple poses to find the index
  int checkTime(float time) {
    for (int i=0;i<_trajectoryTime.length;i++){
      if (time<_trajectoryTime[i]) {
        return i-1;
      }
    }
    return _trajectoryTime.length-1;
  }
  
  // Gets the reference tip velocity in mm and degrees
  float[] getTipVelocity(float time) {
    println("getTipVelocity Time: " + time);
    float[] parameterVelocity = new float[4];
    parameterVelocity = getVelocity(time);
    int i = checkTime(time);
    return new float[] {parameterVelocity[0],parameterVelocity[1],parameterVelocity[2],(parameterVelocity[3])*_rotationAxis[i].x,(parameterVelocity[3])*_rotationAxis[i].y,(parameterVelocity[3])*_rotationAxis[i].z};
  }
  
    
  
  void DebugPrint() {
    for (int i=0;i<_perameters;i++){
      print("\nCurves "+i+":\n");
      Mat.print(_quinticCurves[i],2);
    }
  }
  
}
