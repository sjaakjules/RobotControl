/* This is used for all the calculations used to manipulate the robot and virtual model.
*/

class Robot {
  // Fields
  private float _l1, _l2, _l3,_l4,_l5, _l6, _beta;
  private float[] _desiredEndEffectorVelocity, _motorAngles;
  private float[][] _endEffectorPosition, _lastEndEffectorPosition, _endEffectorVelocity,_lastEndEffectorVelocity,_lastMotorVelocity;
  private Dh[] _links;
  private TransformMatrix _H01, _H02, _H03, _H04, _H05, _H06, _H0t, _H0te, _H36;
  private Motor[] _motors;
  private boolean _isPluggedIn, _isMoving;
  private Jacobian _jac;
  private Control _positionController;

  // Contructors
    // Used for uncalibrated robot
  Robot() {
    initialise();
    _l1 = -140;
    _l2 = 155;
    _l3 = 200;
    _l4 = 150;
    _l5 = 27;
    _l6 = 60;
    _beta = 30;    
    _links[0] = new Dh(0,_l1,0,_l2,true);                    // H01
    _links[1] = new Dh(90,0,_beta,0,true);                  // H12
    _links[2] = new Dh(180,_l3,_beta+90,0,true);             // H23
    _links[3] = new Dh(0,_l4,0,0,true);                     // H34
    _links[4] = new Dh(-90,0,90,0,true);                   // H45
    _links[5] = new Dh(90,0,-90,0,true);                   // H56
    _links[6] = new Dh(0,0,0,_l5,false);                    // H6t   This is from wrist centre to tip of last motor
    _links[7] = new Dh(0,0,0,_l6,false);                    // Htte  This is from last motor tip to end-effector tip
    setup();
  }
  
    // Used for calibrated robot where DH_vars are hard coded in
  Robot(float[][] DH_vars, float numLinks) {
    initialise();
    for (int i = 0;i<numLinks;i++) {
      _links[i] = new Dh(DH_vars[i]);
    }    
    setup();
  }
  
  // Properties
  TransformMatrix H01() {return _H01;}
  TransformMatrix H02() {return _H02;}
  TransformMatrix H03() {return _H03;}
  TransformMatrix H04() {return _H04;}
  TransformMatrix H05() {return _H05;}
  TransformMatrix H06() {return _H06;}
  TransformMatrix H0t() {return _H0t;}
  TransformMatrix H0te() {return _H0te;}
  TransformMatrix H36() {return _H36;}
  boolean isPluggedIn() { return _isPluggedIn; }
  boolean isMoving() { return _isMoving; }
  Dh[] Links() { return _links; }
  Motor[] motors() { return _motors; }
  float[] endPosition() { return Mat.column(_endEffectorPosition,0); }
  float[] endVelocity() { return Mat.column(_endEffectorVelocity,0); }
  Jacobian J() { return _jac; }
  float[] motorAngles() { return _motorAngles; }
  Control posControl() { return _positionController; }
  
  void isMoving(boolean update) { _isMoving = update; }
  
    
  // Methods
  private void initialise() {
    _links = new Dh[8];
    _motors = new Motor[6];
    _endEffectorPosition = new float[3][1];
    _endEffectorVelocity = new float[3][1];
    _lastEndEffectorPosition = new float[3][1];
    _lastEndEffectorVelocity = new float[3][1];
    _isPluggedIn = false;
    _isMoving = false;
    _desiredEndEffectorVelocity = new float[6];
    _lastMotorVelocity = new float[6][1];
    _motorAngles = new float[6];
    _positionController = new Control();    
}

  private void setup() {
        // Create motor instances
    for (int i=0;i<6;i++) {
      _motors[i] = new Motor(i);
    }    
    updateH0i();
    _jac = new Jacobian();    
  }  
  
  void update() {
    // Updates the last angle to measured or previous desired
    for (int i=0;i<6;i++) {
    _motors[i].refreshAngle();
    _lastMotorVelocity[i][0] = _motors[i].velocity();
    _motorAngles[i] = _motors[i].angle();
    }
    
    // Set the new DH angles from feedback or previous desired
    for (int i=0;i<_links.length;i++) {
      if (_links[i].isRotating()) {
        _links[i].updateTheta(_motors[i].angle());
      }
    }
    updateH0i();
    
    _jac.update();
    
    
}


  private void updateH0i() {      
      // The .H().H() where there first gets the TransformMatrix of the DH and the second gets the float[][] within the TransformMatrix object
      _H01 = new TransformMatrix(_links[0].H().H());
      _H02 = new TransformMatrix(Mat.multiply(_H01.H(),_links[1].H().H()));
      _H03 = new TransformMatrix(Mat.multiply(_H02.H(),_links[2].H().H()));
      _H04 = new TransformMatrix(Mat.multiply(_H03.H(),_links[3].H().H()));
      _H05 = new TransformMatrix(Mat.multiply(_H04.H(),_links[4].H().H()));
      _H06 = new TransformMatrix(Mat.multiply(_H05.H(),_links[5].H().H()));
      _H0t = new TransformMatrix(Mat.multiply(_H06.H(),_links[6].H().H()));
      _H0te = new TransformMatrix(Mat.multiply(_H0t.H(),_links[7].H().H()));
      _H36 = new TransformMatrix(Mat.multiply(_links[3].H().H(),Mat.multiply(_links[4].H().H(),_links[5].H().H())));
      _lastEndEffectorPosition = _endEffectorPosition;
      _endEffectorPosition = _H0t.posCol();
      _lastEndEffectorVelocity = _endEffectorVelocity;
      _endEffectorVelocity = Mat.divide(Mat.subtract(_endEffectorPosition,_lastEndEffectorPosition),system.frame_sec());
  }
  
  void setVelocities(float[][] desiredVelocities) {
    print("\n Desired velocities:\n");
    Mat.print(desiredVelocities,2);
    
  //  float[][] errorDot = Mat.sum(desiredVelocities,Mat.transpose(new float[]{);
    for (int i=0;i<_motors.length;i++) {
      _motors[i].angle((_motors[i].angle()+degrees(desiredVelocities[i][0])*system.frame_sec()));
    }
  }
  
  void printMotorAngles(float xpos, float ypos) {
    textAlign(LEFT, TOP);
    for (int i=0;i<_motors.length;i++) {
      text("\nAng: " + String.format("%.1f",_motors[i].angle()),xpos,ypos+20*i);
      text("\nVel: " + String.format("%.1f",_motors[i].velocity()),xpos+100,ypos+20*i);
    }
  }
  
}



  //-------------------------------------------------------------------------------------------------------------------------------
  //-------------------------------------------------------------------------------------------------------------------------------
// Dh class, this is how the matrix is calculated and updated. This represents one line of the DH table.
// Can get jsut rotation vectors and position vectors but for 3D model manipulation a 4x4 matrix is required.
class Dh {
  // Fields
  private TransformMatrix H;
  private float angle, alpha, a, theta, d;
  private boolean isRotating;
  
  // Constructors
// This creates a Dh object, needs the 4 values from a row in the DH table, (alpha, a, theta, d)
  Dh (float alph,float a1,float thet,float d1, boolean rotating) {
    isRotating = rotating;
    alpha = radians(alph);
    theta = radians(thet);
    a = a1;
    d = d1;
    angle = 0;
    H = new TransformMatrix();
    H.setH( cos(theta+angle)           ,    -sin(theta+angle)       ,      0     ,      a       ,
            sin(theta+angle)*cos(alpha), cos(alpha)*cos(theta+angle), -sin(alpha), -d*sin(alpha),
            sin(theta+angle)*sin(alpha), cos(theta+angle)*sin(alpha),  cos(alpha), d*cos(alpha) );
  }
  
    Dh (float[] dh_param) {
    isRotating = boolean((int)dh_param[4]);
    alpha = radians(dh_param[0]);
    theta = radians(dh_param[2]);
    a = dh_param[1];
    d = dh_param[3];
    angle = 0;    
    H = new TransformMatrix();
    H.setH( cos(theta+angle)           ,    -sin(theta+angle)       ,      0     ,      a       ,
            sin(theta+angle)*cos(alpha), cos(alpha)*cos(theta+angle), -sin(alpha), -d*sin(alpha),
            sin(theta+angle)*sin(alpha), cos(theta+angle)*sin(alpha),  cos(alpha), d*cos(alpha) );
  }
  
  // Properties
  TransformMatrix H() { return H; }
  boolean isRotating() { return isRotating;}
  
 // Methods

  // As angle (the angle of the motor with 0 acting as home position) is a float changing angle will not change the 
  // H matrix so it has to be re-calculated. I dont know how to make it a static in this language but this works....
  void updateTheta(float newAngle) {
    angle = radians(newAngle);
    H.setH( cos(theta+angle)           ,    -sin(theta+angle)       ,      0     ,      a       ,
            sin(theta+angle)*cos(alpha), cos(alpha)*cos(theta+angle), -sin(alpha), -d*sin(alpha),
            sin(theta+angle)*sin(alpha), cos(theta+angle)*sin(alpha),  cos(alpha), d*cos(alpha) );
  }
}



  //-------------------------------------------------------------------------------------------------------------------------------
  //-------------------------------------------------------------------------------------------------------------------------------
// Jacobian class, this is how the jacobian is calculated and updated. It is dependent on the following being created and updated:
//      -Robot.Links
//      -Robot.H0i
class Jacobian {
  // Fields
  private float[][] _J,_r6t,_r5t,_r4t,_r3t,_r2t,_r1t,_Jr1,_Jr2,_Jr3,_Jr4,_Jr5,_Jr6,_Jw1,_Jw2,_Jw3,_Jw4,_Jw5,_Jw6, _z;
  
  //Constructors
  Jacobian() {
    _J = new float[6][6];
    _z = new float[][]{{0},{0}, {1}};
  }
  
  // Properties
  float[][] J() { return _J; }
  // Methods
  void update() {
    // vector Pe-Pi from link i to end effector tip in local coordinates
    _r6t = robot.Links()[6].H().posCol();//Mat.sum(robot.Links()[6].H().posCol(),robot.Links()[7].H().posCol());
    _r5t = Mat.sum(robot.Links()[5].H().posCol(),Mat.multiply(robot.Links()[5].H().rotation(),_r6t));
    _r4t = Mat.sum(robot.Links()[4].H().posCol(),Mat.multiply(robot.Links()[4].H().rotation(),_r5t));
    _r3t = Mat.sum(robot.Links()[3].H().posCol(),Mat.multiply(robot.Links()[3].H().rotation(),_r4t));
    _r2t = Mat.sum(robot.Links()[2].H().posCol(),Mat.multiply(robot.Links()[2].H().rotation(),_r3t));
    _r1t = Mat.sum(robot.Links()[1].H().posCol(),Mat.multiply(robot.Links()[1].H().rotation(),_r2t));
    
    // Translational component in origin coordinates
    _Jr6 = Mat.multiply(robot.H06().rotation(),Cross(_z,_r6t));
    _Jr5 = Mat.multiply(robot.H05().rotation(),Cross(_z,_r5t));  
    _Jr4 = Mat.multiply(robot.H04().rotation(),Cross(_z,_r4t)); 
    _Jr3 = Mat.multiply(robot.H03().rotation(),Cross(_z,_r3t)); 
    _Jr2 = Mat.multiply(robot.H02().rotation(),Cross(_z,_r2t));
    _Jr1 = Mat.multiply(robot.H01().rotation(),Cross(_z,_r1t));
    
    // Rotational component in origin coordinates
    _Jw6 = Mat.subMatrix(robot.H06().H(),0,2,2,2);
    _Jw5 = Mat.subMatrix(robot.H05().H(),0,2,2,2);  
    _Jw4 = Mat.subMatrix(robot.H04().H(),0,2,2,2);  
    _Jw3 = Mat.subMatrix(robot.H03().H(),0,2,2,2);  
    _Jw2 = Mat.subMatrix(robot.H02().H(),0,2,2,2);
    _Jw1 = Mat.subMatrix(robot.H01().H(),0,2,2,2);
    
    
    addJocRow(0,_Jr1,_Jr2,_Jr3,_Jr4,_Jr5,_Jr6);
    addJocRow(3,_Jw1,_Jw2,_Jw3,_Jw4,_Jw5,_Jw6);
  }
  
  void addJocRow(int start, float [][] q1, float [][] q2, float [][] q3, float [][] q4, float [][] q5, float [][] q6) {
    for (int i=0;i<3;i++) {
      _J[i+start]=Mat.concat(q1[i],Mat.concat(q2[i],Mat.concat(q3[i],Mat.concat(q4[i],Mat.concat(q5[i],q6[i])))));
    }
  }
  
  void printJoc(float xpos, float ypos) {
    textFont(gui.display(), 20);
    text("Jacobian:",xpos,ypos);
    textFont(gui.display(), 18);
    textAlign(RIGHT,TOP);
    text(String.format("%.2f", _J[0][0]) + "\n"+ String.format("%.2f", _J[1][0]) + "\n"+ String.format("%.2f", _J[2][0]) + "\n"+ String.format("%.2f", _J[3][0]) + "\n"+ String.format("%.2f", _J[4][0]) + "\n"+ String.format("%.2f", _J[5][0]),xpos,ypos+20 );
    text(String.format("%.2f", _J[0][1]) + "\n"+ String.format("%.2f", _J[1][1]) + "\n"+ String.format("%.2f", _J[2][1]) + "\n"+ String.format("%.2f", _J[3][1]) + "\n"+ String.format("%.2f", _J[4][1]) + "\n"+ String.format("%.2f", _J[5][1]),xpos+100,ypos+20 );
    text(String.format("%.2f", _J[0][2]) + "\n"+ String.format("%.2f", _J[1][2]) + "\n"+ String.format("%.2f", _J[2][2]) + "\n"+ String.format("%.2f", _J[3][2]) + "\n"+ String.format("%.2f", _J[4][2]) + "\n"+ String.format("%.2f", _J[5][2]),xpos+200,ypos+20 );
    text(String.format("%.2f", _J[0][3]) + "\n"+ String.format("%.2f", _J[1][3]) + "\n"+ String.format("%.2f", _J[2][3]) + "\n"+ String.format("%.2f", _J[3][3]) + "\n"+ String.format("%.2f", _J[4][3]) + "\n"+ String.format("%.2f", _J[5][3]),xpos+300,ypos+20 );
    text(String.format("%.2f", _J[0][4]) + "\n"+ String.format("%.2f", _J[1][4]) + "\n"+ String.format("%.2f", _J[2][4]) + "\n"+ String.format("%.2f", _J[3][4]) + "\n"+ String.format("%.2f", _J[4][4]) + "\n"+ String.format("%.2f", _J[5][4]),xpos+400,ypos+20 ); 
    text(String.format("%.2f", _J[0][5]) + "\n"+ String.format("%.2f", _J[1][5]) + "\n"+ String.format("%.2f", _J[2][5]) + "\n"+ String.format("%.2f", _J[3][5]) + "\n"+ String.format("%.2f", _J[4][5]) + "\n"+ String.format("%.2f", _J[5][5]),xpos+500,ypos+20 );
  }
  
}

  //-------------------------------------------------------------------------------------------------------------------------------
  //-------------------------------------------------------------------------------------------------------------------------------


