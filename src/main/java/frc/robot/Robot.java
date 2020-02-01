/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;

//Motor Controllers
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //Motor Controllers
  WPI_TalonFX topRightDrivey, topLeftDrivey, bottomRightDrivey, bottomLeftDrivey; 

  DifferentialDrive drivey;
  SpeedControllerGroup leftDrivey, rightDrivey;

  Timer time;

  Joystick logitech;

  //Ran when robot first starts
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    time = new Timer();

    topLeftDrivey = new WPI_TalonFX(7);
    topRightDrivey = new WPI_TalonFX(5);
    bottomLeftDrivey = new WPI_TalonFX(8);
    bottomLeftDrivey = new WPI_TalonFX(6);

    leftDrivey = new SpeedControllerGroup(topLeftDrivey, bottomLeftDrivey);
    rightDrivey = new SpeedControllerGroup(topRightDrivey, bottomRightDrivey);

    drivey = new DifferentialDrive(leftDrivey, rightDrivey);

    logitech = new Joystick(0);

  }

  
  @Override
  public void robotPeriodic() {
  }

  //Run when auto first begins
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    time.reset();
    time.start();
  }

  //Called periodically during auto
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        while (time.get() < 2.0) {
          drivey.arcadeDrive(0.5, 0.0);
        }
        break;
    }
  }

  //Called periodically during teleop
  @Override
  public void teleopPeriodic() {
    
    
    
  }

  //Called periodically during testing
  @Override
  public void testPeriodic() {
  }
}
