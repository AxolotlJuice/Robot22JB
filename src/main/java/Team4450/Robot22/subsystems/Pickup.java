package Team4450.Robot22.subsystems;

import static Team4450.Robot22.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import Team4450.Lib.Util;
import Team4450.Lib.ValveDA;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pickup extends SubsystemBase{
    private WPI_VictorSPX           upperVictor, lowerVictor;

    private MotorControllerGroup    pickupDrive;

    private ValveDA                 pickupValve = new ValveDA(PICKUP_VALVE);

    private double          pickupPower = .35;
    private boolean         extended = false, pickupRunning = false;

    //creates pickupDrive which can be used 
    public Pickup() {
        Util.consoleLog();

        lowerVictor = new WPI_VictorSPX(LOWER_PICKUP_VICTOR);
        upperVictor = new WPI_VictorSPX(UPPER_PICKUP_VICTOR);

        lowerVictor.setInverted(true);
        upperVictor.setInverted(true);

        pickupDrive = new MotorControllerGroup(lowerVictor, upperVictor);

        retract();

        Util.consoleLog("Pickup Created");
    }

    //skipped "ball eye" stuff because we don't have that on the account that we don't have a "ball eye"


    //dashboard stuff
    private void updateDS() {
        SmartDashboard.putBoolean("Pickup", pickupRunning);
        SmartDashboard.putBoolean("PickupExtended", extended);
    }

    //extends pickup
    public void extend() {
        Util.consoleLog();

        pickupValve.SetB();

        extended = true;

        start(pickupPower);
    }

    //retracts pickup
    public void retract() {
        Util.consoleLog();

        pickupValve.SetA();

        extended = false;

        stop();
    }

    // don't know how to describe this one but it like retracts the pickup and extends it
    public void toggleDeploy() {
        Util.consoleLog("%b", isExtended());

        if(isExtended())
            retract();
        else
            extend();
    }

    //starts the motor for pickup
    private void start(double power) {
        Util.consoleLog("%.2f", power);

        pickupDrive.set(power);

        pickupRunning = true;

        updateDS();
    }

    //stops the motor for pickup
    private void stop() {
        Util.consoleLog();

        pickupDrive.stopMotor();

        pickupRunning = false;

        updateDS();
    }

    //says if things are extended?
    public boolean isExtended() {
        return extended;
    }

    //says things are running?
    public boolean isRunning() {
        return pickupRunning;
    }
    //created by AJD do not steal or i will seek legal action

	public void initialize()
	{
		Util.consoleLog();

        retract();
	}
}