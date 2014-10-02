import remixlab.proscene.*;
import papaya.*;
import controlP5.*;
import processing.serial.*;
import org.gwoptics.graphics.graph2D.Graph2D;
import org.gwoptics.graphics.graph2D.LabelPos;
import org.gwoptics.graphics.graph2D.traces.Line2DTrace;
import org.gwoptics.graphics.graph2D.traces.ILine2DEquation;
import java.util.*;

System system;
GUI gui;
Model model;
Robot robot;
Master_Program mp;
PApplet embed;

void setup() {
  embed = this;
  system = new System();
  gui = new GUI();
  controlSetup();
  robot = new Robot();
  model = new Model();
  mp = new Master_Program();
 // xposs = new graph(500,360,200,100);
}

void draw() {
  system.update();
  robot.update();
  model.draw();
  mp.draw();
  gui.draw();
  print("\n Angles: ");
  Mat.print(robot.motorAngles(),2);
//  xposs.draw();
  
}


