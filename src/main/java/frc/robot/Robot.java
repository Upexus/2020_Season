package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;

//NetworkTable
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
  //Vars used by the whole class, put here to only init once.
  boolean search = false;
  
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
    //Auto Stuff
    time     = new Timer();
    logitech = new Joystick(0);

    //Mechanisms
    shooterAlpha = new WPI_TalonFX(9);
    shooterBeta  = new WPI_TalonFX(10);
    shooter      = new SpeedControllerGroup(shooterAlpha, shooterBeta);
    shooter        .setInverted(true);
    turret       = new CANSparkMax(11, MotorType.kBrushless);
    turretSensor = new CANEncoder(turret);

    //Drive
    topLeftDrivey    = new WPI_TalonFX(7);
    topRightDrivey   = new WPI_TalonFX(5);
    bottomLeftDrivey = new WPI_TalonFX(8);
    bottomLeftDrivey = new WPI_TalonFX(6);
    leftDrivey  = new SpeedControllerGroup(topLeftDrivey, bottomLeftDrivey);
    rightDrivey = new SpeedControllerGroup(topRightDrivey, bottomRightDrivey);
    drivey = new DifferentialDrive(leftDrivey, rightDrivey);

    //NetworkTable stuffs
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
  }
  
  @Override
  public void robotPeriodic() {
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically for debugging purposes
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }

  @Override
  public void autonomousInit() {
    
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopPeriodic() {
    //Drive
    drivey.arcadeDrive(logitech.getRawAxis(1), logitech.getRawAxis(4));

    //Shooter
    if (logitech.getRawButton(8)) {
      shooter.set(0.5);
    } else {
      shooter.set(0.0);
    }

    //Turret
    if (logitech.getRawButtonPressed(1)) {
      search = !search;
    } 

    if (search) {
      limelightChecker();
    }

    //have thing for turret for possible manual control
  }

  @Override
  public void testPeriodic() {

  }

  public void limelightChecker() {
    double tx = table.getEntry("tx").getDouble(0.0); //x axis offset
    double tv = table.getEntry("tv").getDouble(0.0); //whether or not we see the goal
    double kp = -0.1;
    double min_command     = 0.05;
    double heading_error   = -tx;                    //x axis fixer
    double turret_adjust   = 0.0;                    //used to adjust the turret
    
    if (tx > 0.0) {
      turret_adjust = kp*heading_error - min_command;//-0.1 * x axis offset - 
    } else if (tx < 0.0) {
      turret_adjust = kp*heading_error + min_command;
    }

    turret.set(turret_adjust);

    if (tv == 0.0) {
      turret_adjust = 0.3;
      turret.set(turret_adjust);
    } else {
      heading_error = tx;
      turret_adjust = kp * heading_error;
      turret.set(turret_adjust);
    }
  }

  public void seeker() {

  }

}