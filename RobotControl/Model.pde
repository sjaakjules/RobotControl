/* This is the model class used for everything pertaining to the virtual model.
*/

class Model {
  
  // Fields
  private PShape[] _objs;
  private PShape _link4, _tool, _base;
  private PGraphics _pg;

  // Constructors
  Model() {
    _objs = new PShape[3];
    _objs[0] = loadShape("Link1.obj");
    _objs[1] = loadShape("Link2.obj");
    _objs[2] = loadShape("Link3.obj");
    _link4 = loadShape("Link4.obj");
    _tool  = loadShape("Tool.obj");
    _base  = loadShape("Base.obj");
    _pg = createGraphics(400, 400, P3D); 
  }
  
  // Properties
  PGraphics getImg() { return _pg; }
  
  // Methods
  // This is all the code to draw the model, it will create an image of the 3D render scene at the end of endDraw() 
  // and store it within 'pg' the Pgraphics object. image(pg,X_loc,Y_loc); will display the image created.
  void draw() {
    _pg.beginDraw();
    _pg.lights();  
    _pg.camera(400,600,600, 0,0,0,0,0,-1);                             //  sets the view for the image using 3 X,Y,Z values,
    _pg.directionalLight(50, 50, 50, 0, 0, -1);
    _pg.background(#6C8A9B);                                           //  camera(position, focus, up vector)
    
    _pg.fill(#FF0009);
    // This is about placing and positioning the various links in the scene correctly, The Denavit-Hartenberg was used as an inbuilt
    // function allowed a 4x4 rotation/translation matrix to be applied! (super easy) 
    // The only issue i have with it is the X axis seems to not follow the DH convention.
    
    // By using the pushMatrix and popMatrix it allows matrix rotations to occur, The pop applies the matricies. 
    // There has been a class called Dh which H01 is an object of, to get the 4x4 matrix H01.H returns a float[Row][Col],
    // I could apply the rotations to each object by applying H02, H03 to link2 and 3 respectively and using pop after each link
    // Howwever using the DH i can apply the relattive transformations in order. 
    // The angle from 0 is updated (within the Dh class and DenHaten tab) and the matricies re-calculated every update! 
    
    _pg.shape(_base);
    
    // applyMatrix is a std processing comand (hence blue colour) and takes float values for all 16 entries.
    for (int i=0;i<3;i++) {
      _pg.pushMatrix();
      _pg.applyMatrix(robot.Links()[i].H().H()[0][0],robot.Links()[i].H().H()[0][1],robot.Links()[i].H().H()[0][2],robot.Links()[i].H().H()[0][3],
                     robot.Links()[i].H().H()[1][0],robot.Links()[i].H().H()[1][1],robot.Links()[i].H().H()[1][2],robot.Links()[i].H().H()[1][3],
                     robot.Links()[i].H().H()[2][0],robot.Links()[i].H().H()[2][1],robot.Links()[i].H().H()[2][2],robot.Links()[i].H().H()[2][3],
                     robot.Links()[i].H().H()[3][0],robot.Links()[i].H().H()[3][1],robot.Links()[i].H().H()[3][2],robot.Links()[i].H().H()[3][3]);
      _pg.shape(_objs[i]);  
    }
    
    _pg.pushMatrix();
    _pg.applyMatrix(robot.H36().H()[0][0],robot.H36().H()[0][1],robot.H36().H()[0][2],robot.H36().H()[0][3],
                   robot.H36().H()[1][0],robot.H36().H()[1][1],robot.H36().H()[1][2],robot.H36().H()[1][3],
                   robot.H36().H()[2][0],robot.H36().H()[2][1],robot.H36().H()[2][2],robot.H36().H()[2][3],
                   robot.H36().H()[3][0],robot.H36().H()[3][1],robot.H36().H()[3][2],robot.H36().H()[3][3]);
    _pg.shape(_link4); 
    
    _pg.pushMatrix();
    _pg.applyMatrix(robot.Links()[6].H().H()[0][0],robot.Links()[6].H().H()[0][1],robot.Links()[6].H().H()[0][2],robot.Links()[6].H().H()[0][3],
                    robot.Links()[6].H().H()[1][0],robot.Links()[6].H().H()[1][1],robot.Links()[6].H().H()[1][2],robot.Links()[6].H().H()[1][3],
                    robot.Links()[6].H().H()[2][0],robot.Links()[6].H().H()[2][1],robot.Links()[6].H().H()[2][2],robot.Links()[6].H().H()[2][3],
                    robot.Links()[6].H().H()[3][0],robot.Links()[6].H().H()[3][1],robot.Links()[6].H().H()[3][2],robot.Links()[6].H().H()[3][3]);
    _pg.shape(_tool);  
    _pg.popMatrix(); 
    _pg.popMatrix();  
    _pg.popMatrix();
    _pg.popMatrix();  
    _pg.popMatrix();  
    
    _pg.endDraw();
  }
}
