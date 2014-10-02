class Master_Program {
  // Fields  
  private Gripper _newPos;
  private float[][] _desiredTipVelocities,_desiredVelocities;
  private float[] _position1,_position2;
  private int _startMoveTime;
  private float _movingTime;
  private TransformMatrix[] poses;
  Trajectory movement;
  
  // Constructors
  Master_Program() {
    _newPos = new Gripper();
    _desiredTipVelocities = new float[1][];
    _desiredVelocities = new float[6][1];
    _position1 = new float[] {0,0,0,0,0,0};
    _position2 = new float[] {0,0,0,0,0,0};
  }
  
  // Properties
  
  // Methods
  void draw() {    
    _newPos.draw();
   buttonControl();
  }
  
  void buttonControl() {    
      float[] t = new float[]{0,float(time.getText())};
      if (!robot.isMoving()){
      poses = new TransformMatrix[]{robot.H0t(),_newPos.H()};
      movement = new Trajectory(poses,t);
      } 
      
    // Home button pressed
    if (home.isMousePressed()) {
      movement = new Trajectory(float(time.getText()),robot.motorAngles(),new float[]{0,0,0,0,0,0});
      movement.DebugPrint();
      _startMoveTime = millis();
      robot.isMoving(true);
      freezeControls();
    }
  // Not moving and go button pressed
    if (!robot.isMoving() && go.isMousePressed()) {
      _startMoveTime = millis();
      robot.isMoving(true);
      freezeControls();
    }
    
  // Point button checks, if not set will set to current, otherwise create move
    if (Mat.norm2(_position1)==0 && point1.isMousePressed()) {
      _position1 = robot.motorAngles();
      Point1_imgs[0] = loadImage("point1b.png");
    } else if (point1.isMousePressed()) {
      movement = new Trajectory(float(time.getText()),robot.motorAngles(),_position1);
      _startMoveTime = millis();
      robot.isMoving(true);
      freezeControls();
    }
    if (Mat.norm2(_position2)==0 && point2.isMousePressed()) {
      _position2 = robot.motorAngles();
      Point2_imgs[0] = loadImage("point2b.png");
    } else if (point2.isMousePressed()) {
      movement = new Trajectory(float(time.getText()),robot.motorAngles(),_position2);
      _startMoveTime = millis();
      robot.isMoving(true);
      freezeControls();
    }
    
  // Is moving
    if (robot.isMoving()) {
      _movingTime = (millis()-_startMoveTime)/(1000.0); 
    // At the end of Move
      if (_movingTime>=float(time.getText())) {
        robot.isMoving(false);
        _movingTime = 0.0;
        unfreezeControls();
        robot.setVelocities(new float[][]{{0},{0},{0},{0},{0},{0}});
    //During move 
      // If movement is issued with 6 motor velocities
      } else if (movement.isMotorSpace()) {
        _desiredVelocities[0] = movement.getVelocity(_movingTime);
        robot.setVelocities(Mat.transpose(_desiredTipVelocities));
      // If movement is issued with end Effector velocities
      } else if (!movement.isMotorSpace()) {
        // The reference end effector velocities [x y z wx wy wz]
        _desiredTipVelocities[0] = movement.getTipVelocity(_movingTime);
      //  print("\nVelocity Tip:\n");
     //   Mat.print(_desiredTipVelocities,2);
        // The reference joint velocites [a1; a2; a3; a4; a5; a6] as column
        _desiredVelocities = Mat.multiply(Mat.inverse(robot.J().J()),Mat.transpose(_desiredTipVelocities));
        // Set the velocites
        robot.setVelocities(_desiredVelocities);
      }
    }
  }
  
}
