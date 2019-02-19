package frc.team2225.robot;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.team2225.robot.command.MoveForward;
import frc.team2225.robot.command.Teleop;
import frc.team2225.robot.subsystem.Drivetrain;
import frc.team2225.robot.subsystem.Elevator;
import frc.team2225.robot.subsystem.RollerIntake;
import frc.team2225.robot.subsystem.UltrasonicSensor;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static Drivetrain drivetrain;
    public static RollerIntake rollerIntake;
    public static UltrasonicSensor ultrasonicSensor;
    public static Elevator elevator;
    public static final int updateFlags = EntryListenerFlags.kUpdate | EntryListenerFlags.kNew | EntryListenerFlags.kImmediate;

    public static ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
    public static ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        drivetrain = new Drivetrain(2, 1, 3, 4, SPI.Port.kOnboardCS0);
        //TODO: Find the IDs for the roller intake motors
        rollerIntake = new RollerIntake(0, 1, 0);
        elevator = new Elevator(10);
        ultrasonicSensor = new UltrasonicSensor(0);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to
     * the switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        new MoveForward().start();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {

    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void teleopInit() {
        new Teleop().start();
    }

    @Override
    public void testInit() {

    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {

    }
}
