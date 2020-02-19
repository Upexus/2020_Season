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
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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
  private WPI_TalonFX frontLeft, frontRight, rearLeft, rearRight, leftShooter, rightShooter;
  private CANSparkMax turret, preroller, vertConvey, indexer, intake;
  private TalonFXSensorCollection shooterSensor;
  private DifferentialDrive drive;
  private SpeedControllerGroup leftDrivey, rightDrivey, shooters;
  private VictorSPX funnel;
  //private VictorSPX colorWheel;
  
  //Misc
  private Timer time;
  private Joystick driver, mechanic;

  //NetworkTable
  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;

  //Sensors
  private DigitalInput bottomCell, middleCell, topCell;

  //Solenoids
  private Solenoid shifters, leftIntake, rightIntake;
  private Compressor compressor;

  @Override
  public void robotInit() {
    //Auto Stuff
    time     = new Timer();
    driver   = new Joystick(0);
    mechanic = new Joystick(1);

    //Mechanisms
    //Shooters
    leftShooter  = new WPI_TalonFX(9);
    rightShooter = new WPI_TalonFX(10);
    leftShooter.setInverted(true);
    shooters = new SpeedControllerGroup(leftShooter, rightShooter);
    
    //Intake and Conveyors
    turret      = new CANSparkMax(13, MotorType.kBrushless);
    preroller   = new CANSparkMax(11, MotorType.kBrushless);
    indexer     = new CANSparkMax(14, MotorType.kBrushless);
    vertConvey  = new CANSparkMax(12, MotorType.kBrushless);
    intake      = new CANSparkMax(15, MotorType.kBrushless);
    funnel      = new VictorSPX(16);

    //color wheel :|
    //colorWheel = new VictorSPX(18);

    //Drive
    frontLeft    = new WPI_TalonFX(5);
    frontRight   = new WPI_TalonFX(6);
    rearLeft     = new WPI_TalonFX(7);
    rearRight    = new WPI_TalonFX(8);
    leftDrivey   = new SpeedControllerGroup(frontLeft, rearLeft);
    rightDrivey  = new SpeedControllerGroup(frontRight, rearRight);
    drive        = new DifferentialDrive(leftDrivey, rightDrivey);

    //NetworkTable stuffs
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");

    //Sensors
    bottomCell = new DigitalInput(0);
    middleCell = new DigitalInput(1);
    topCell    = new DigitalInput(2);

    //pneumatics
    shifters    = new Solenoid(0);
    leftIntake  = new Solenoid(1);
    rightIntake = new Solenoid(2);
    compressor  = new Compressor();
  }
  
  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("Top", topCell.get());
    SmartDashboard.putBoolean("Mid", middleCell.get());
    SmartDashboard.putBoolean("Bot", bottomCell.get());

    SmartDashboard.putNumber("Velocity", shooterSensor.getIntegratedSensorVelocity());
  
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
  public void teleopInit() {
    leftIntake.set(true);
    rightIntake.set(true);
  }

  @Override
  public void teleopPeriodic() {
    //Drive
      drive.arcadeDrive(driver.getRawAxis(1), driver.getRawAxis(4));

      //Shooter
      if (driver.getRawButton(5)) {
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

      SmartDashboard.putBoolean("Auto Search", search);

      //Preroller
      if (driver.getRawButton(6)) {
        preroller.set(1);
      } else {
        preroller.set(0);
      }
      
      //Intake
      if (driver.getRawAxis(2) > .6) {
        intake.set(1);
      } else {
        intake.set(0);
      }

      //Shifters
      if (driver.getPOV() == 0) {
        shifters.set(true);
      } else if (driver.getPOV() == 4) {
        shifters.set(false);
      }

    //Mechanics
      if (mechanic.getPOV() == 0) {
        vertConvey.set(1);
      } else if (mechanic.getPOV() == 4) {
        vertConvey.set(-1);
      } else {
        vertConvey.set(0);
      }

      if (mechanic.getPOV() == 2) {
        indexer.set(1);
      } else if (mechanic.getPOV() == 6) {
        indexer.set(-1);
      } else {
        indexer.set(0);
      }

      if (mechanic.getRawButton(1) == true) {
        funnel.set(ControlMode.PercentOutput, 100);
      } else {
        funnel.set(ControlMode.PercentOutput, 0);
      }
      
      if(mechanic.getRawButton(3) == true) {
        leftIntake.set(true);
        rightIntake.set(true);
      } else if (mechanic.getRawButton(4) == true) {
        leftIntake.set(false);
        rightIntake.set(false);
      }
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
