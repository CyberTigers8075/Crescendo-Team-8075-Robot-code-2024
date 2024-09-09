// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Scorpion extends SubsystemBase {
  /** Creates a new Scorpion. */
  
    public DoubleSolenoid scorpionPiston1, scorpionPiston2;
;
    //DoubleSolenoid flapPiston;
    Compressor compressor;
    public Boolean lift;
    

  public Scorpion(){
    compressor = new Compressor(Constants.MODULE_NUMBER, PneumaticsModuleType.CTREPCM);
    scorpionPiston1 = new DoubleSolenoid(Constants.MODULE_NUMBER, PneumaticsModuleType.CTREPCM, Constants.FORWARD_CHANNEL_TWO, Constants.REVERSE_CHANNEL_TWO);
    //scorpionPiston2 = new DoubleSolenoid(Constants.MODULE_NUMBER, PneumaticsModuleType.CTREPCM, Constants.FORWARD_CHANNEL_TWO, Constants.REVERSE_CHANNEL_TWO);
    //flapPiston = new DoubleSolenoid(Constants.MODULE_NUMBER, PneumaticsModuleType.CTREPCM, Constants.FORWARD_CHANNEL_TWO, Constants.REVERSE_CHANNEL_TWO);
    
    lift = false;
  }

  public void liftUp(){
    scorpionPiston1.set(kForward);
    lift = true;
  }

  public void liftDown(){
    scorpionPiston1.set(kReverse);
    lift = false;
  }
  
  public void liftOff(){
    scorpionPiston1.set(kOff);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
