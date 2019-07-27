package frc.robot.tools.math;
import java.lang.Math;

public class Vector {

   private double xVec;
   private double yVec;
   public Vector() {
	  xVec =0;
	  yVec = 0.0;
   }
   public Vector( double x, double y ) {
	  xVec = x;
	  yVec= y;
   }
   public double dot( Vector v1 ) {
	  return xVec*v1.getxVec() + yVec*v1.getyVec();
   }
   public double length() {
	  return Math.sqrt ( xVec*xVec + yVec*yVec );
   }
   public double getxVec(){
	  return xVec;
   }
   public double getyVec(){
	  return yVec;
   }
   public void setX(double x){
	  xVec = x;
   }
   public void setY(double y){
	  yVec = y;
   }
} 