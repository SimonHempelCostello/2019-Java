/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.ButtonMap;
import frc.robot.RobotConfig;
import frc.robot.RobotMap;
import frc.robot.RobotStats;
import frc.robot.sensors.DriveEncoder;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private double deadZone = 0.00;
	private double turn =0;
	private double throttel = 0;
	private double povValue;
	private double ratio = 0;
  private double sensitivity;
  public static DriveEncoder leftMainDrive = new DriveEncoder(RobotMap.leftDriveLead,RobotMap.leftDriveLead.getSelectedSensorPosition(0));
  public static DriveEncoder rightMaindrive = new DriveEncoder(RobotMap.rightDriveLead,RobotMap.rightDriveLead.getSelectedSensorPosition(0));


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void setLowGear(){
    RobotMap.shifters.set(RobotMap.lowGear);
  }

  public void setHighGear(){
    RobotMap.shifters.set(RobotMap.highGear);
  }
  public void arcadeDrive(){
    double leftPower;
    double rightPower;
    throttel = ButtonMap.getDriveThrottle()*0.9; 
		ratio = Math.abs(throttel);
		povValue = ButtonMap.getPOV();
		turn = ButtonMap.getRotation();
		if(Math.abs(throttel)>RobotStats.joyStickDeadZone){
			leftPower = (throttel - (sensitivity*turn*ratio));
			rightPower = (throttel + (sensitivity*turn*ratio));
		}
		else{
			leftPower = (-turn)*sensitivity;
			rightPower = (turn)*sensitivity; 
		}
		if(ButtonMap.quickTurn()) {
			leftPower = throttel +(-turn);
			rightPower= throttel +(turn);
		}
		if(Math.abs(leftPower)>1) {
			leftPower = (leftPower/Math.abs(leftPower));
			rightPower = Math.abs(rightPower/leftPower)*(rightPower/Math.abs(rightPower));
		}
		else if(Math.abs(rightPower)>1) {
			rightPower = (rightPower/Math.abs(rightPower));
			leftPower = Math.abs(leftPower/rightPower)*(leftPower/Math.abs(leftPower));
    }
    RobotMap.leftDriveLead.set(ControlMode.PercentOutput, leftPower);
    RobotMap.rightDriveLead.set(ControlMode.PercentOutput, rightPower);
		if(ButtonMap.shiftDown()){
			setLowGear();
		}
		else if(ButtonMap.shiftUp()) {
			setHighGear();
		}
		if(RobotMap.shifters.get() == RobotMap.highGear) {
				sensitivity =2.25;
		}
		else if(RobotMap.shifters.get() == RobotMap.lowGear) {
				sensitivity =1.25;
    }
  }

  public void stopDriveTrainMotors(){
    for(TalonSRX talon : RobotMap.driveMotorLeads){
        talon.set(ControlMode.PercentOutput, 0);
    }
  }
}
