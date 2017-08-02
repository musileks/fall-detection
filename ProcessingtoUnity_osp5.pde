import processing.serial.*;
import de.bezier.data.sql.*;
import oscP5.*;
import netP5.*;

OscP5 oscP5;
NetAddress myRemoteLocation;

MySQL ms;
java.sql.Timestamp last_ts;

// database information
String database = "falldetection", user = "root", pass = "";
float yaw,pitch,roll;

void setup() 
{
  size(200, 200); //create box for showing result    
    ms = new MySQL( this, "localhost", database, user, pass ); //mysql setting
    last_ts = new java.sql.Timestamp( 0 ); 

  oscP5 = new OscP5(this, 9000);
  myRemoteLocation = new NetAddress("127.0.0.1", 8000);
}

void draw() {
  background(0);
  
   if ( ms.connect() )
    {
        ms.query( "SELECT Yaw,Pitch,Roll FROM test WHERE Time > '"+last_ts+"' ORDER BY Time ASC" );
        while( ms.next() )
        {
             yaw = ms.getFloat("Yaw");
             pitch = ms.getFloat("Pitch");
             roll = ms.getFloat("Roll");
        }
    
    }
      //value of Yaw Pitch Roll from sensor data 
      text("degree value axe x : " + roll, 20, 30); //30
      text("degree value axe y : " + pitch, 20, 80); //80
      text("degree value axe z : " + yaw, 20,130); //130
     
      //send message to unity through /mpu6050
      OscMessage myMessage = new OscMessage("/mpu6050");
      
      myMessage.add(yaw); 
      myMessage.add(pitch); 
      myMessage.add(roll); 

      oscP5.send(myMessage, myRemoteLocation);
       
}