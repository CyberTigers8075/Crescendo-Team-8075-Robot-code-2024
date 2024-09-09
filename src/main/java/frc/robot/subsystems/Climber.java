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
import edu.wpi.first.wpilibj.Servo;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public VictorSPX climberMotor;
  public Servo servo, servo2;
  public boolean up;
  
  int currentPosition = 0; // Current Position Climber thinks it at the bottom
  int wantedPosition = 0; // Position Climber wants to be in
    
  public Climber() {
    servo  = new Servo(1);
    servo2 = new Servo(0);
    up = true;
    climberMotor = new VictorSPX(Constants.CLIMBER_MOTOR);
    climberMotor.setNeutralMode(NeutralMode.Brake);

      
  }

  public double deadBand(double power){
    if(Math.abs(power)<0.09){
      return 0;
    }else{
      return power;
    }
  }

  public void climberMovement(){
    int dPadInput = RobotContainer.mechJoystick.getPOV();
    //System.out.println(dPadInput);
    if (dPadInput != -1){
      if(dPadInput >= 315 || dPadInput<= 45 || dPadInput == 0){
        climberMotor.set(ControlMode.PercentOutput, -.99);
      }
      else if(dPadInput >= 135 && dPadInput <= 225){
        climberMotor.set(ControlMode.PercentOutput, 0.8);
      }

      else{
        climberMotor.set(ControlMode.PercentOutput, 0.0);
      }
    }else{
      climberMotor.set(ControlMode.PercentOutput, 0.0);
    }
    }

    
  public void servoMovement(){
    
    if(up){
      servo.setAngle(0.0);
      servo2.setAngle(0.0);
      up = false;
    }else{
      servo.setAngle(270);
      servo2.setAngle(-90.0);
      
    }

  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

