/* This is a class used to be the mid between the robot and the virtual model.
    The angDes and angMeas are the variables read and written to communicate between
    processing and arduino. Inclusion of velocity and acceleration is for future development.
    -- Note the fields are private to protect accidental manipulation of the variables --
    */
class Motor {
  // Fields
  private float _desiredAngle, _desiredVelocity, _desiredAcceleration, 
                _measuredAngle, _measuredVelocity, _measuredAcceleration,
                _lastAngle, _lastVelocity, _lastAcceleration;
  private int _id;
  
  // Contructors
  Motor(int id) {
    _id = id;
  }
  
  // Properties
  void angle(float newAng) {_desiredAngle = newAng;}
  float angle() {return _lastAngle;}
  float velocity() {return _lastVelocity; }
    
  // Methods
  // This updates the desired velocity and desired position
  void velocity(float newVelocity) {
    _desiredVelocity = newVelocity;
    _desiredAngle=_lastAngle+newVelocity*system.frame_sec();
  }
  
    // Updates angLast from measured or desired depending if robot is connected.
  void refreshAngle() {
    if (robot.isPluggedIn()) {        
      _measuredVelocity = _lastVelocity+(_measuredAngle-_lastAngle)/(system.frame_sec());
      _measuredAcceleration = _lastAcceleration+(_measuredVelocity-_lastVelocity)/(system.frame_sec());
      _lastAcceleration = _measuredAcceleration;
      _lastVelocity = _measuredVelocity;
      _lastAngle = _measuredAngle;
    } else {
      _lastAcceleration = _desiredAcceleration;
      _lastVelocity = _desiredVelocity;
      _lastAngle = _desiredAngle;
    }
  }
  
  void UpdateAngles(float angle) {
    _measuredAngle = angle;
  }
  
}

  
