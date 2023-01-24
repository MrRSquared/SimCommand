// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPIDCMD extends CommandBase {
  /** Creates a new ElevatorPIDCMD. */

  private final ElevatorSubsystem elevatorSubsystem;
  private final double setpoint;
  private final PIDController m_Controller;
  private static final double kElevatorKp = 5.0;

  public ElevatorPIDCMD(ElevatorSubsystem elevatorSubsystem, double setpoint) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.setpoint = setpoint;
    this.m_Controller = new PIDController (kElevatorKp, 0, 0);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidOutput = m_Controller.calculate(elevatorSubsystem.getEncoderDistance(), Units.inchesToMeters(setpoint));
    m_Controller.setTolerance(Units.inchesToMeters(10)); //Add tolerance to prevent oscillations from locking up the command in an infinite loop
    elevatorSubsystem.setMotorVoltage(pidOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setMotorVoltage(0.0);
    System.out.println("Elevator Set has ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("ElevatorCMD is finishing");
    return m_Controller.atSetpoint();
    
  }
}
