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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ButtonMap;
import frc.robot.RobotConfig;
import frc.robot.RobotMap;
import frc.robot.RobotStats;
import frc.robot.sensors.DriveEncoder;
import frc.robot.tools.controlLoops.VelocityPID;

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
	private double minTurnFactor = 0.4;
  public static DriveEncoder leftMainDrive = new DriveEncoder(RobotMap.leftDriveLead,RobotMap.leftDriveLead.getSelectedSensorPosition(0));
  public static DriveEncoder rightMainDrive = new DriveEncoder(RobotMap.rightDriveLead,RobotMap.rightDriveLead.getSelectedSensorPosition(0));
	private double speed;
  private double f = 0.332;
  private double p = 0.75;
  private double i = 0.00001;
  private double d = 7.50;
  private int profile = 0;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void setLowGear(){
    RobotMap.shifters.set(RobotMap.lowGear);
  }
	public void initVelocityPIDs(){
    RobotMap.leftDriveLead.selectProfileSlot(profile, 0);
    RobotMap.leftDriveLead.config_kF(profile, f, 0);
    RobotMap.leftDriveLead.config_kP(profile, p, 0);
    RobotMap.leftDriveLead.config_kI(profile, i, 0);
    RobotMap.leftDriveLead.config_kD(profile, d, 0);
    RobotMap.leftDriveLead.set(ControlMode.Velocity, leftMainDrive.convertftpersToNativeUnitsper100ms(speed));
    RobotMap.rightDriveLead.selectProfileSlot(profile, 0);
    RobotMap.rightDriveLead.config_kF(profile, f, 0);
    RobotMap.rightDriveLead.config_kP(profile, p, 0);
    RobotMap.rightDriveLead.config_kI(profile, i, 0);
    RobotMap.rightDriveLead.config_kD(profile, d, 0);
    RobotMap.rightDriveLead.set(ControlMode.Velocity, rightMainDrive.convertftpersToNativeUnitsper100ms(speed));

	}
  public void setHighGear(){
    RobotMap.shifters.set(RobotMap.highGear);
  }
	public void arcadeDrive2(){
		double leftPower;
		double rightPower;
		double differential;
		
		throttel = ButtonMap.getDriveThrottle(); 
		if(throttel ==0){
			throttel = 0.001;
		}
		ratio = Math.abs(1/throttel);
		povValue = ButtonMap.getPOV();
		turn = ButtonMap.getRotation();
		differential = (turn*ratio*sensitivity) + Math.abs(minTurnFactor*turn);

		leftPower = (throttel - (differential));
		rightPower = (throttel + (differential));
	
		if(Math.abs(leftPower)>1) {
			rightPower = Math.abs(rightPower/leftPower)*Math.signum(rightPower);
			leftPower = Math.signum(leftPower);
		}
		else if(Math.abs(rightPower)>1) {
			leftPower = Math.abs(leftPower/rightPower)*Math.signum(leftPower);
			rightPower = Math.signum(rightPower);
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
				sensitivity =1;
		}
		else if(RobotMap.shifters.get() == RobotMap.lowGear) {
				sensitivity =1;
    }
	}
	public void autoHatchPickup(){
		
	}
	public void setLeftSpeed(double speed){
		SmartDashboard.putNumber("output", leftMainDrive.getVelocity());
		SmartDashboard.putNumber("error", RobotMap.leftDriveLead.getClosedLoopError());
		SmartDashboard.putNumber("target", RobotMap.leftDriveLead.getClosedLoopTarget());
		SmartDashboard.putNumber("position", leftMainDrive.getDistance());
		RobotMap.leftDriveLead.set(ControlMode.Velocity, leftMainDrive.convertftpersToNativeUnitsper100ms(speed));
	}
	public void setRightSpeed(double speed){
		RobotMap.rightDriveLead.set(ControlMode.Velocity, rightMainDrive.convertftpersToNativeUnitsper100ms(speed));

	}
  public void stopDriveTrainMotors(){
    for(TalonSRX talon : RobotMap.driveMotorLeads){
        talon.set(ControlMode.PercentOutput, 0);
    }
  }
}
