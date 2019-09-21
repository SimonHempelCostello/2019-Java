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
        return -oi.driverController.getRawAxis(1);
    } 
    public static double getRotation(){
        return oi.driverController.getRawAxis(4);
    }
    public static int getPOV(){
        return oi.driverController.getPOV();
    }
    public static boolean shiftUp(){
        return oi.driverController.getBumper(Hand.kLeft);
    }
    public static boolean shiftDown(){
        return oi.driverController.getBumper(Hand.kRight);
    }
    public static boolean quickTurn(){
        return oi.driverController.getRawAxis(3)>0.5;
    }
    public static boolean testVelocity(){
        return oi.driverController.getXButton();
    }
    public static boolean autoBreakTapeTracking(){
        return oi.driverController.getBButton();
    }
    public static boolean switchCamera(){
        return oi.driverController.getTriggerAxis(Hand.kLeft)>0.2;
    }
    public static boolean armUp(){
        return oi.operatorController.getYButton();  
    }
    public static boolean armOuttake(){
        return oi.operatorController.getXButton();  
    }
    public static boolean armResting(){
        return oi.operatorController.getAButton();  
    }
    public static double armManualControlValue(){
        return oi.operatorController.getRawAxis(1);
    }
    public static boolean grabHatch(){
        return oi.operatorController.getBumper(Hand.kLeft);
    }
    public static boolean releaseHatch(){
        return oi.operatorController.getBumper(Hand.kRight);
    }
    public static boolean outTakeBall(){
        return (oi.operatorController.getRawAxis(3)>0.5);
    }
    public static boolean inTakeBall(){
        return (oi.operatorController.getRawAxis(2)>0.5);
    }

    
    

}
