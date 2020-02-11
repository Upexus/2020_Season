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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;

//NetworkTable shit
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

//Motor Controllers
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANEncoder;


public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  //Limelight
  private boolean LimelightHasValidTarget = false;
  private double LimelightDriveCommand = 0.0;
  private double LimelightSteerCommand = 0.0;

  //Motor Controllers
  private WPI_TalonFX topRightDrivey, topLeftDrivey, bottomRightDrivey, bottomLeftDrivey; 
  private WPI_TalonFX shooterAlpha, shooterBeta; 
  private CANSparkMax turret;
  private CANEncoder turretSensor;
  private SpeedControllerGroup leftDrivey, rightDrivey, shooter;
  private TalonFXSensorCollection shooterSensor;
  private DifferentialDrive drivey;

  //Misc
  private Timer time;
  private Joystick logitech;

  //NetworkTable
  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;

  /* Notes:
  https://docs.limelightvision.io/en/latest/cs_aiming.html AIMING
  https://docs.limelightvision.io/en/latest/cs_estimating_distance.html DISTANCE ESTIMATION
  https://docs.limelightvision.io/en/latest/getting_started.html#wiring BASIC PROGRAMMING AND WIRING AND SHIT
  */

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    time = new Timer();

    shooterAlpha = new WPI_TalonFX(9);
    shooterBeta  = new WPI_TalonFX(10);
    shooter.setInverted(true);
    shooter = new SpeedControllerGroup(shooterAlpha, shooterBeta);
    turret = new CANSparkMax(11, MotorType.kBrushless);
    turretSensor = new CANEncoder(turret);

    topLeftDrivey    = new WPI_TalonFX(7);
    topRightDrivey   = new WPI_TalonFX(5);
    bottomLeftDrivey = new WPI_TalonFX(8);
    bottomLeftDrivey = new WPI_TalonFX(6);

    leftDrivey  = new SpeedControllerGroup(topLeftDrivey, bottomLeftDrivey);
    rightDrivey = new SpeedControllerGroup(topRightDrivey, bottomRightDrivey);

    drivey = new DifferentialDrive(leftDrivey, rightDrivey);

    logitech = new Joystick(0);

    //NetworkTable stuffs
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
  }
  
  @Override
  public void robotPeriodic() {
    //read values periodically
    double x    = tx.getDouble(0.0);
    double y    = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
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
    aimLimelight();    
  }

  //Called periodically during teleop
  @Override
  public void teleopPeriodic() {
    if (logitech.getRawButton(1) == true) {
      shooter.set(0.5);
    } else {
      shooter.set(0.0);
    }

    if (logitech.getRawButtonPressed(9)) {
      turret.set(-0.5);
    } else if (logitech.getRawButtonPressed(10)) {
      turret.set(0.5);
    }
  }

  //Called periodically during testing
  @Override
  public void testPeriodic() {
  }

  public void aimLimelight() {
    double kp = -0.1;
    double min_command = 0.2;

    double tx = table.getEntry("tx").getDouble(0);
    
    double heading_error = -tx;
    double steering_adjust = 0.0;
    if (tx > 1.0) {
      steering_adjust = kp*heading_error - min_command;
      time.reset();
      turret.set(steering_adjust);
    } else if (tx < 1.0) {
      steering_adjust = kp*heading_error  + min_command;
      turret.set(-steering_adjust);
    }    
  }

  public void seeker() {
    double kp = -0.1;
    double tv = table.getEntry("tv").getDouble(0);
    double tx = table.getEntry("tx").getDouble(0);

    double steering_adjust = 0.0;

    if (tv == 0.0) {
      steering_adjust = 0.3;
      turret.set(steering_adjust);
    } else {
      double heading_error = tx;
      steering_adjust = kp * tx;
      turret.set(steering_adjust);
    }
  }
}
