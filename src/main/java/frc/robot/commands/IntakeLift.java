// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
// Scorpion Command 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Scorpion;

public class IntakeLift extends Command {
  private final Scorpion scorpion;
  private int count;
  private boolean lifted;
  
  /** Creates a new IntakeLift. */
  public IntakeLift(Scorpion sc) {
    scorpion = sc;
    count = 0;
    addRequirements(scorpion);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //lifted = scorpion.lift;
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
      scorpion.liftUp();

  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    scorpion.liftOff();
    System.out.println("off");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
