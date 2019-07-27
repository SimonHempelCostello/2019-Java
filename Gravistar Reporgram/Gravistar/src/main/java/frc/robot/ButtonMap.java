/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * Add your docs here.
 */
public class ButtonMap {
    public static OI oi = new OI();
    public static double getDriveThrottle(){
        return -oi.xboxController0.getRawAxis(1);
    } 
    public static double getRotation(){
        return oi.xboxController0.getRawAxis(4);
    }
    public static int getPOV(){
        return oi.xboxController0.getPOV();
    }
    public static boolean shiftUp(){
        return oi.xboxController0.getBumper(Hand.kLeft);
    }
    public static boolean shiftDown(){
        return oi.xboxController0.getBumper(Hand.kRight);
    }
    public static boolean quickTurn(){
        return oi.xboxController0.getRawAxis(3)>0.5;
    }
    public static boolean armUp(){
        return oi.xboxController1.getYButton();  
    }
    public static boolean armOuttake(){
        return oi.xboxController1.getXButton();  
    }
    public static boolean armResting(){
        return oi.xboxController1.getAButton();  
    }
    public static double armManualControlValue(){
        return oi.xboxController1.getRawAxis(1);
    }
}
