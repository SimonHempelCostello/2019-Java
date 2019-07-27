package frc.robot.sensors;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.RobotConfig;
import frc.robot.RobotStats;

public class DriveEncoder {
	private TalonSRX masterTalon;
	private int startingValue;
	
	public DriveEncoder(TalonSRX talon, int startingValue) {
		masterTalon = talon;
	}
	public double getEncoderValue() {
		return masterTalon.getSelectedSensorPosition(0)-startingValue;
	}
	public double getEncoderVelocity(){
		return masterTalon.getSelectedSensorVelocity(0);
	}
	public double getDistance(){
		return ((((getEncoderValue())/RobotStats.encoderTicsPerWheelRotation)*RobotStats.wheelCircum)/12);
	}
	public double getVelocity(){
		return (((((getEncoderVelocity()*10))/RobotStats.encoderTicsPerWheelRotation)*RobotStats.wheelCircum)/12);
	}
	public void softReset(){
		startingValue = masterTalon.getSelectedSensorPosition(0);
	}
	public double convertftpersToNativeUnitsper100ms(double feetPerSecond){
		return (((feetPerSecond/10)*(12))/(RobotStats.wheelCircum))*RobotStats.encoderTicsPerWheelRotation;
	}

}
