// Read angle values robot.motorAngles()   it is a float[]
// Write angle values robot.motorsupdateAngles(float angle) 

// The serial port:
Serial myPort;       
String dataReading;
// The angle reading strings
String[] pot;
String gpmsg;

void ardusetup(){
  // Initilise angle reading strings
  for (int i=0;i<6;i++) {
    pot[i] = "";
  }
  dataReading = "";
  gpmsg = "waiting";
  
  // List all the available serial ports:
  println(Serial.list());
  
}

void ardudraw(){

}

// The following bit of code manages the incoming angles reported by the arduino.
void serialEvent(Serial myPort) {
 dataReading = myPort.readString();
 if(dataReading!=null){
   
    char inByte = dataReading.charAt(0); 
   // println(inByte);
    switch(inByte) 
      {
      case 'a':
        robot.motors()[0].UpdateAngles(float(dataReading.substring(1)));
        break;
      case 'b':
        robot.motors()[1].UpdateAngles(float(dataReading.substring(1)));
        break;
     case 'c':
        robot.motors()[2].UpdateAngles(float(dataReading.substring(1)));
        break;
     case 'd':
        robot.motors()[3].UpdateAngles(float(dataReading.substring(1)));
        break;
     case 'e':
        robot.motors()[4].UpdateAngles(float(dataReading.substring(1)));
        break;
     case 'f':
        robot.motors()[5].UpdateAngles(float(dataReading.substring(1)));
        break;        
      case 'A':
        pot[0] = dataReading.substring(1);
        break;
      case 'B':
        pot[1] = dataReading.substring(1);
        break;
     case 'C':
        pot[2] = dataReading.substring(1);
        break;
     case 'D':
        pot[3] = dataReading.substring(1);
        break;
     case 'E':
        pot[4] = dataReading.substring(1);
        break;   
     case 'F':
        pot[5] = dataReading.substring(1);
        break;  
     case 'Z':
        gpmsg = dataReading.substring(1);
        break;    
      }
    
  } 
}



