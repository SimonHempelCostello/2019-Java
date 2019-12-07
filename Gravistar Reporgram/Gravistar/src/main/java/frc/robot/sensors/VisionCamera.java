package frc.robot.sensors;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.tools.math.Point;
import edu.wpi.first.hal.util.UncleanStatusException;

public class VisionCamera {
   JSONParser parser = new JSONParser();
   SerialPort port;
   public String sanatizedString = "nothing";
   public double lastParseTime;
   private double distance;
   private double angle;
   private double badAngle = -100.0;
   private double badDistance = -11.0;
   private Point targetPoint;
   
   
   public VisionCamera(SerialPort jevois) {
       port = jevois;
       targetPoint = new Point(0,0);
    }
   
   public void updateVision(){
      try{
         String unsantString = this.getString();
         String jsonString = unsantString.substring(unsantString.indexOf('{'), unsantString.indexOf('}')+1);
         double tryDistance = badDistance;
         double tryAngle = badAngle;

         if (jsonString != null){

            tryDistance = parseDistance(jsonString);
            tryAngle = parseAngle(jsonString);
         }
         if (tryAngle != badAngle){
            distance = tryDistance;
            angle = tryAngle;

            lastParseTime = Timer.getFPGATimestamp();
         }
         
      }
      catch (Exception e){
      }
   }
   public double parseAngle(String jsonString){
 
      try{
         Object object = parser.parse(jsonString);   
         JSONObject jsonObject = (JSONObject) object;
         if (jsonObject != null){
            double distString = (double) jsonObject.get("Angle");
            return Double.valueOf(distString);
         }     
      } 
         
      catch(ParseException e) {
      } 
      catch(UncleanStatusException e) {
      } 
      catch(ClassCastException e) {
      }
               
      return badAngle;
   }
 
   public double parseDistance(String jsonString){
      try{
         Object object = parser.parse(jsonString);
         JSONObject jsonObject = (JSONObject) object;
         if (jsonObject != null){ 
            double distString = (double) jsonObject.get("Distance");
            return (Double.valueOf(distString))/12;
         }
      }   
      catch(ParseException e) {
      } 
      catch(UncleanStatusException e) {
      } 
      catch(ClassCastException e) {      
      }
       return badDistance; 
      } 
   public double getDistance(){   
      return distance;
   }
   public double getAngle(){
      return angle;
   }
   public String getString(){
      try {    
         if(port.getBytesReceived()>2){
            String unsanatizedString = port.readString();
            if(unsanatizedString.length()>5&&!unsanatizedString.isBlank()&&!unsanatizedString.isEmpty()){
               sanatizedString = unsanatizedString;
            }
         }
      } 
      catch (Exception e) {
      }
      return sanatizedString;   
   }
   public Point getTargetPoint(){
      double xAverage = 0;
      double yAverage = 0;
      updateVision();
      for(int i = 0; i<20; i++){
         xAverage=xAverage+(getDistance()*Math.cos(Math.toRadians(getAngle())));
         yAverage=yAverage+(getDistance()*Math.sin(Math.toRadians(getAngle())));
      }
      xAverage =xAverage/20;
      yAverage = yAverage/20;
      targetPoint.setLocation(xAverage, yAverage);
      return targetPoint;
   }
}