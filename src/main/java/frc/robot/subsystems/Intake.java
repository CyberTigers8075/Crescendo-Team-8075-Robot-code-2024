// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.RobotContainer;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public VictorSPX intakeMotor;
  

  public Intake() {
  intakeMotor = new VictorSPX(Constants.INTAKE_MOTOR);
  intakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double deadBand(double power){
    if(Math.abs(power)<0.09){
      return 0;
    }else{
      return power;
    }
  }

public void intakeMovement(){
  if(RobotContainer.mechJoystick.getRawAxis(Constants.LT)!= 0){//retracts
    double intakePower = deadBand(RobotContainer.mechJoystick.getRawAxis(Constants.LT)*0.8);
    intakeMotor.set(ControlMode.PercentOutput, intakePower);
    
  }else if(RobotContainer.mechJoystick.getRawAxis(Constants.RT)!= 0){//extends
    double intakePower = deadBand(RobotContainer.mechJoystick.getRawAxis(Constants.RT)*0.8);
    intakeMotor.set(ControlMode.PercentOutput, -intakePower);}
  }
}

