package frc.robot.commands.controls;

import frc.robot.RobotMap;

import com.ctre.phoenix.CANifier;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ChangeLightColor{
	private double red;
	private double green;
	private double blue;
	private CANifier canifier;

    public ChangeLightColor(double R, double G, double B, CANifier chosenCANifier) {
    	red=R;
    	green = G;
		blue = B;
		canifier = chosenCANifier;
		
    	canifier.setLEDOutput(green,CANifier.LEDChannel.LEDChannelA);
		canifier.setLEDOutput(blue,CANifier.LEDChannel.LEDChannelB);
		canifier.setLEDOutput(red,CANifier.LEDChannel.LEDChannelC);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }
    public void changeLedColor(double R,double G,double B) {
    	red=R;
    	green = G;
    	blue = B;
    	canifier.setLEDOutput(green,CANifier.LEDChannel.LEDChannelA);
		canifier.setLEDOutput(blue,CANifier.LEDChannel.LEDChannelB);
		canifier.setLEDOutput(red,CANifier.LEDChannel.LEDChannelC);
    	
    }
}