// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.XRPDrivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPReflectanceSensor;
import edu.wpi.first.wpilibj2.command.Command;

/** Line follower command that uses the drivetrain subsystem. */
public class LineFollow extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")

  private final XRPDrivetrain m_drivetrain;
  private final XRPReflectanceSensor m_sensor = new XRPReflectanceSensor();

  //PID & Control Parameters
  //Variables with dimensions are in S.I. units.
  /* Wing:
   * P 2.5
   * I 0.00
   * D kp/0.030
   * SPEED 0.50
   * 
   * Centre
   * P 3.5
   * I 0.0
   * D kp/0.025
   * SPEED 0.40
   */
  private final double kp = 3.5;
  private final double ki = 0.0;
  private final double kd = kp/0.025;
  private final double speed = 0.40;

  private double m_error;
  private double m_previousError;
  private double m_t;
  private double m_distance = 0.0;

  private final double m_threshold = 200;

  public LineFollow(XRPDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
    m_drivetrain.resetEncoders();
    //subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  //PID Compute Function
  public double computePIDResponse() {
    double dt = this.m_t - Timer.getFPGATimestamp();
    double resp = this.kp * this.m_error;
    resp += this.ki * (this.m_error - this.m_previousError)/dt;
    resp += this.kd * 0.5*(this.m_error + this.m_previousError)*dt;

    this.m_previousError = this.m_error;
    this.m_t = Timer.getFPGATimestamp();
    return resp;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Rough method for tracking distance.
    SmartDashboard.putNumber("error", m_error);
    this.m_distance = (m_drivetrain.getLeftDistanceInch() + m_drivetrain.getRightDistanceInch())/2;
    if(this.m_distance >= this.m_threshold) {
      //m_drivetrain.arcadeDrive(0.0, 0.0);
      //return;
    }

    //Actual PID line following part.
    double lReflectance = m_sensor.getLeftReflectanceValue();
    double rReflectance = m_sensor.getRightReflectanceValue();
    this.m_error = (lReflectance - rReflectance)/1.0;
    //Ideally, the error should be zero, i.e. difference = 0
    //If the error > 0, Right is in black, left in in white = turn right : CW
    //If the error < 0, Right is in white, left is in black = turn left : CCW
    double pid_response = computePIDResponse();
    SmartDashboard.putNumber("pid response", pid_response);
    //m_drivetrain.tankDrive(pid_response, 1-pid_response);
    m_drivetrain.arcadeDrive(speed, pid_response);
    /*
     * Arcade drive inverse kinematics for differential drive platform:
     * Parameters:
     * xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
     * zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is positive.
     * squareInputs If set, decreases the input sensitivity at low speeds.
     */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
