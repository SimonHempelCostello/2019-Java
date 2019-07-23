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
        return -oi.xboxController.getRawAxis(1);
    } 
    public static double getRotation(){
        return -oi.xboxController.getRawAxis(4);
    }
    public static int getPOV(){
        return oi.xboxController.getPOV();
    }
    public static boolean shiftUp(){
        return oi.xboxController.getBumper(Hand.kLeft);
    }
    public static boolean shiftDown(){
        return oi.xboxController.getBumper(Hand.kRight);
    }
    public static boolean quickTurn(){
        return oi.xboxController.getRawAxis(3)>0.5;
    }

}
