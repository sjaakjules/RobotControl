class Gripper {
  // Fields
  private PShape _tool;
  private TransformMatrix _S;
  private Quaternion _Sq;
  
  // Constructors
  Gripper() {
    _tool  = loadShape("Tool.obj");
    _S = new TransformMatrix();
    _Sq = new Quaternion();
  }
  
  // Properties
  TransformMatrix H() { return _S; }
  
  // Methods
  void draw() {
    _S.rotation(getXYZrotation(rX1.getValue()+180, (rY1.getValue()), (rZ1.getValue())));
    _Sq.fromRotationMatrix(_S.rotation());
    _S.pos(pX1.getValue(),pY1.getValue(),pZ1.getValue());
    model.getImg().beginDraw();
    
    model.getImg().lights();  
    model.getImg().camera(400,600,600, 0,0,0,0,0,-1);                             //  sets the view for the image using 3 X,Y,Z values,
    model.getImg().directionalLight(50, 50, 50, 0, 0, -1);                                         //  camera(position, focus, up vector)
    
    model.getImg().fill(#FF0009);
    
    
    model.getImg().pushMatrix();  
    model.getImg().applyMatrix(_S.H()[0][0],_S.H()[0][1],_S.H()[0][2],_S.H()[0][3],
                   _S.H()[1][0],_S.H()[1][1],_S.H()[1][2],_S.H()[1][3],
                   _S.H()[2][0],_S.H()[2][1],_S.H()[2][2],_S.H()[2][3],
                   0      ,0      ,0      ,1);
    model.getImg().shape(_tool);
    model.getImg().popMatrix();
    model.getImg().endDraw();
  }
}
