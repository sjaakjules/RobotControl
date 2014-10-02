/* This class is used to handle all GUI functionality. It is shown above the global content for ease of access.
  */
class GUI {
  // Fields 
  private PFont _title, _display;
  
  // Constructors
  GUI () {
    size(1280,720,P2D);                                      // Size of the whole aplication
    frameRate(60);                                           // Framerate but cant seem to be faster tha int startTime;
    _title = loadFont("PresaANTIPIXEL.COM.AR-48.vlw");      // Various fonts used to display (tools>create font to add your own)
    _display = loadFont("SegoeUIEmoji-20.vlw");    
    //GraphSetup(500,360,300,100);
  }
  
  // Properties
  PFont display() {return _display;}
  PFont title() {return _title;}

  // Methods
  void draw() {  
    background(#93D1E0);
    textFont(_display, 15);
    text(int(frameRate),1250,20);
    textFont(_title,48);
    fill(#036AA0);
    textAlign(CENTER, TOP);
    text("Robotics  Test  Environment", 640,10);
    robot.H0te().printLocation(200,100);
    robot.printMotorAngles(140,180);
    robot.J().printJoc(700, 100);
   // buttonControl();
   image(model.getImg(),0,360);
  }

  
}

/*  The following is all the ControlP5 objects, sliders, knobs and buttons
    where they are all global variables to allow all classes access. 
    -- The setup is called in the constructor of GUI --
    */
    
// Global Fields
ControlP5 cp5;
Textfield time;
Textfield[] gain;
Slider pX1, pY1, pZ1, rX1,rY1,rZ1;
Button go, home,point1,point2;
PImage[] Point1_imgs,Point2_imgs,Move_imgs,Home_imgs;

// Global Methods
void controlSetup() {
  smooth();
  noStroke();
  gain = new Textfield[6];
  cp5 = new ControlP5(this);  
  gain[0] = cp5.addTextfield("Gain_1")
                 .setPosition(20,100)
                 .setSize(100,25)
                 .setFont(gui.display())
                 .setColor(color(#004E74))
                 .setAutoClear(false)
                 .setColorBackground(color(#004E74,30))
                 .setText("0")
                 ;
  gain[1] = cp5.addTextfield("Gain_2")
                 .setPosition(20,140)
                 .setSize(100,25)
                 .setFont(gui.display())
                 .setColor(color(#004E74))
                 .setAutoClear(false)
                 .setColorBackground(color(#004E74,30))
                 .setText("0")
                 ;
  gain[2] = cp5.addTextfield("Gain_3")
                 .setPosition(20,180)
                 .setSize(100,25)
                 .setFont(gui.display())
                 .setColor(color(#004E74))
                 .setAutoClear(false)
                 .setColorBackground(color(#004E74,30))
                 .setText("0")
                 ;
  gain[3] = cp5.addTextfield("Gain_4")
                 .setPosition( 20,220)
                 .setSize(100,25)
                 .setFont(gui.display())
                 .setColor(color(#004E74))
                 .setAutoClear(false)
                 .setColorBackground(color(#004E74,30))
                 .setText("0")
                 ;
  gain[4] = cp5.addTextfield("Gain_5")
                 .setPosition( 20,260)
                 .setSize(100,25)
                 .setFont(gui.display())
                 .setColor(color(#004E74))
                 .setAutoClear(false)
                 .setColorBackground(color(#004E74,30))
                 .setText("0")
                 ;
  gain[5] = cp5.addTextfield("Gain_6")
                 .setPosition( 20,300)
                 .setSize(100,25)
                 .setFont(gui.display())
                 .setColor(color(#004E74))
                 .setAutoClear(false)
                 .setColorBackground(color(#004E74,30))
                 .setText("0")
                 ;
  pX1 =  cp5.addSlider("Start X position")
           .setPosition(300,100)
           .setRange(0,210)
           .setValue(40) 
           ;
  pY1 =  cp5.addSlider("Start Y position")
           .setPosition(300,120)
           .setRange(-150,150)
           .setValue(-130)            
           ;
  pZ1 =  cp5.addSlider("Start Z position")
           .setPosition(300,140)
           .setRange(0,210)
           .setValue(165)  
           ;
  rX1 =  cp5.addSlider("Start X rotation")
           .setPosition(300,160)
           .setRange(-90,90)
           .setValue(0) 
           ;
  rY1 =  cp5.addSlider("Start Y rotation")
           .setPosition(300,180)
           .setRange(-90,90)
           .setValue(0) 
           ;
  rZ1 =  cp5.addSlider("Start Z rotation")
           .setPosition(300,200)
           .setRange(-90,90)
           .setValue(0) 
           ;
           
  time = cp5.addTextfield("Time (sec)")
            .setPosition(500,100)
            .setSize(100,25)
            .setFont(gui.display())
            .setColor(color(#004E74))
            .setAutoClear(false)
            .setColorBackground(color(#004E74,30))
            .setText(str(1)) 
            ;
  PImage[] Move_imgs = {loadImage("move1.png"),loadImage("move2.png"),loadImage("move3.png")};
  go = cp5.addButton("go")
     .setValue(0)
     .setPosition(500,150)
     .setImages(Move_imgs)
     .updateSize()
     ;
     
  PImage[] Home_imgs = {loadImage("home1.png"),loadImage("home2.png"),loadImage("home3.png")};
  home = cp5.addButton("home")
     .setValue(0)
     .setPosition(560,150)
     .setImages(Home_imgs)
     .updateSize()
     ;
     
  PImage[] Point1_imgs = {loadImage("point1a.png"),loadImage("point1d.png"),loadImage("point1c.png")};
  point1 = cp5.addButton("point1")
     .setValue(0)
     .setPosition(500,210)
     .setImages(Point1_imgs)
     .updateSize()
     ;
  PImage[] Point2_imgs = {loadImage("point2a.png"),loadImage("point2d.png"),loadImage("point2c.png")};
  point2 = cp5.addButton("point2")
     .setValue(0)
     .setPosition(560,210)
     .setImages(Point2_imgs)
     .updateSize()
     ;
     
  cp5.addButton("Relax")
     .setPosition(50,350)
     .setSize(200,50)
     .setValue(0)
     ;
     
  cp5.addButton("GOpen")
     .setValue(100)
     .setPosition(50,420)
     .setSize(80,50)
     ;

  cp5.addButton("GClose")
     .setValue(100)
     .setPosition(150,420)
     .setSize(80,50)
     ;
}

void Gain_1(String theValue) {
  robot.posControl()._gain[0] = float(theValue);
  
}

void freezeControls() {
  pX1.lock() ;
  pY1.lock() ;
  pZ1.lock() ;
  rX1.lock() ;
  rY1.lock() ;
  rZ1.lock() ;
  time.lock();
  go.lock() ;
}
void unfreezeControls() {
  pX1.unlock() ;
  pY1.unlock() ;
  pZ1.unlock() ;
  rX1.unlock() ;
  rY1.unlock() ;
  rZ1.unlock() ;
  time.unlock();
  go.unlock() ;
}


