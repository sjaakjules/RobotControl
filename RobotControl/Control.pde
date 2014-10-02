class Control {
  // Fields
  private float[] _gain;
  private float[] _error;
  
  // Constructors
  Control() {
    _gain = new float[6];
    _error = new float[6];
  }
  // Properties
  
  // Methods
  float[] getControlEffort(float[] reference, float[] measure) {
    updateGain();
    return Mat.multiply(Mat.subtract(reference,measure),_gain);
  }
  
  float[] getControlEffort(float[] reference, float[] measure, float[] gain) {
    _gain = gain;
    return Mat.multiply(Mat.subtract(reference,measure),_gain);
  }
  
  float[] getControlEffort(TransformMatrix positionRef, TransformMatrix positionMeasure, float[] velocityRef) {
    updateGain();
    float[] nRef = Mat.column(positionRef.rotation(),0);
    float[] oRef = Mat.column(positionRef.rotation(),1);
    float[] aRef = Mat.column(positionRef.rotation(),2);
    float[] nMea = Mat.column(positionMeasure.rotation(),0);
    float[] oMea = Mat.column(positionMeasure.rotation(),1);
    float[] aMea = Mat.column(positionMeasure.rotation(),2);
    _error = concat(Mat.subtract(positionRef.pos(),positionMeasure.pos()),                            // Translation Error
    Mat.multiply(Mat.sum(Mat.sum(Mat.cross(nMea,nRef),Mat.cross(oMea,oRef)),Mat.cross(aMea,aRef)),0.5));  // Rotation Error
    return Mat.sum(Mat.multiply(_gain,_error),velocityRef);
  }
  
  void updateGain() {
    for (int i=0;i<_gain.length;i++) {
      _gain[i] = float(gain[i].getText());
    }
  }
}
