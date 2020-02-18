package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;

//Drive
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

//Auto
import edu.wpi.first.wpilibj.Timer;

//Output
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Input
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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.DigitalInput;

//Pnuematics 
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;


public class Robot extends TimedRobot {
  //Variables
  boolean search = false;

  //Motor Controllers
  private WPI_TalonFX frontLeft, frontRight, rearLeft, rearRight;
  private WPI_TalonFX leftShooter, rightShooter;
  private CANSparkMax turret;
  private TalonFXSensorCollection shooterSensor;
  private DifferentialDrive drive;
  private SpeedControllerGroup leftDrivey, rightDrivey, shooters;

  //Misc
  private Timer time;
  private Joystick driver, mechanic;

  //NetworkTable
  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;

  //Sensors
  private CANEncoder turretSensor;
  private DigitalInput bottomCell, middleCell, topCell;

  /* Notes:
  https://docs.limelightvision.io/en/latest/cs_aiming.html AIMING
  https://docs.limelightvision.io/en/latest/cs_estimating_distance.html DISTANCE ESTIMATION
  https://docs.limelightvision.io/en/latest/getting_started.html#wiring BASIC PROGRAMMING AND WIRING
  */

  @Override
  public void robotInit() {
    //Auto Stuff
    time     = new Timer();
    driver = new Joystick(0);
    mechanic = new Joystick(1);

    //Mechanisms
    leftShooter = new WPI_TalonFX(9);
    rightShooter = new WPI_TalonFX(10);
    // Change which shooter is inverted in the case that both rotate in the wrong direction.
    leftShooter.setInverted(true);
    shooters = new SpeedControllerGroup(leftShooter, rightShooter);

    turret = new CANSparkMax(11, MotorType.kBrushless);
    turretSensor = new CANEncoder(turret);

    //Drive
    frontLeft    = new WPI_TalonFX(5);
    frontRight   = new WPI_TalonFX(6);
    rearLeft = new WPI_TalonFX(7);
    rearRight = new WPI_TalonFX(8);
    leftDrivey  = new SpeedControllerGroup(frontLeft, rearLeft);
    rightDrivey = new SpeedControllerGroup(frontRight, rearRight);
    drive = new DifferentialDrive(leftDrivey, rightDrivey);

    //NetworkTable stuffs
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");

    //Sensors
    bottomCell = new DigitalInput(0);
    middleCell = new DigitalInput(1);
    topCell = new DigitalInput(2);
  }
  
  @Override
  public void robotPeriodic() {

  }

  @Override
  public void autonomousInit() {
    
  }

  @Override
  public void autonomousPeriodic() {
    //read values periodically
    //double x = tx.getDouble(0.0);
    //double y = ty.getDouble(0.0);
    //double area = ta.getDouble(0.0);
  }

  @Override
  public void teleopPeriodic() {
    //Drive
    drive.arcadeDrive(driver.getRawAxis(1), driver.getRawAxis(4));

    //Shooter
    if (driver.getRawButton(8)) {
      shooters.set(1);
    } else {
      shooters.set(0.0);
    }

    //Turret
    if (driver.getRawButtonPressed(1)) {
      search = !search;
    } 

    if (search) {
      limelight();
    }

    //have thing for turret for possible manual control
  }

  @Override
  public void testPeriodic() {

  }

  public void limelight() {
    double tx = table.getEntry("tx").getDouble(0.0); //x axis offset
    double tv = table.getEntry("tv").getDouble(0.0); //whether or not we see the goal
    double kp = -0.1;
    double min_command     = 0.05;
    double heading_error   = -tx;                    //x axis fixer
    double turret_adjust   = 0.0;                    //used to adjust the turret
    
    if (tx > 0.0) {
      turret_adjust = kp*heading_error - min_command;
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
}