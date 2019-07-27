/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.RobotStats;
import frc.robot.sensors.ArmEncoder;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private double kp = 4;
  private double ki = 0.001;
  private double kd = 1;
  private double kf = 0.45574;
  private int acceleration = 1500;
  private int velocity = 750;
  
  public static ArmEncoder mainArmEncoder = new ArmEncoder(RobotMap.armMaster);

  @Override
  public void initDefaultCommand() {
  }
  public void initializeArmControl(){
    RobotMap.armMaster.configMotionCruiseVelocity(velocity);
    RobotMap.armMaster.configMotionAcceleration(acceleration);
    RobotMap.armMaster.config_kP(0, kp);
    RobotMap.armMaster.config_kI(0, ki);
    RobotMap.armMaster.config_kD(0, kd);
    RobotMap.armMaster.config_kF(0, kf);
    RobotMap.armMaster.selectProfileSlot(0, 0);

  }
  public void setArmPercentPower(double value){
    RobotMap.armMaster.set(ControlMode.PercentOutput,value);
  }
  public void setArmPostion(double angle){
    if(angle ==150&&RobotMap.arm.mainArmEncoder.getAngle()>145){
      keepArmUp();
    }
    if(angle ==0&&RobotMap.arm.mainArmEncoder.getAngle()<5){
      keepArmDown();
    }
    RobotMap.armMaster.set(ControlMode.MotionMagic, RobotMap.arm.mainArmEncoder.convertAngleToEncoderTics(angle));


    SmartDashboard.putNumber("error", RobotMap.armMaster.getClosedLoopError(0));
    SmartDashboard.putNumber("output", RobotMap.armMaster.getMotorOutputPercent());
    SmartDashboard.putNumber("desiredAnlge", RobotMap.armMaster.getClosedLoopTarget(0));

  }
 
  public void keepArmUp(){
    setArmPercentPower(0.1);
  }
  public void keepArmDown(){
    setArmPercentPower(-0.1);
  }


  public void zeroArmUP(){
    mainArmEncoder.setForwardLimitSwitchAngle();
  }
  public void zeroArmDown(){
    mainArmEncoder.setReverseLimitSwitchAngle();
  }

}
