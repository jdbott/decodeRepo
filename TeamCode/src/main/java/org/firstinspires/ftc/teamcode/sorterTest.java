package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

/*******************************************************************
|                                                                  |
|                                                                  |
intake      Position 1         Position 2        Position 3      turret
|                                                                  |
|                                                                  |
 *****************************************************************/

@TeleOp(name = "sorterTest")
public class sorterTest extends LinearOpMode {
    private Limelight3A limelight;
    private BHI260IMU imu;
    private Turret turret;
    private Gantry gantry;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);

        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
        imu.resetYaw();

        turret = new Turret();
        turret.init(hardwareMap, "turretMotor", DcMotorSimple.Direction.FORWARD);

        gantry = new Gantry(hardwareMap);
        
        waitForStart();

        limelight.start();


        int motif = 0; // 21 for GPP, 22 for PGP, 23 for PPG, 0 for undetected

        // Intake balls and store order from color sensor in a variable
        // 1 = GPP
        // 2 = PGP
        // 3 = PPG
        // 0 = all else
        int intake = 0;

        switch(motif) {
            case 21: // GPP motif
                switch (intake) {
                    case 1: //GPP intake
                        // fire position 1
                        // fire position 2
                        // fire position 3
                        break;
                    case 2: // PGP intake
                        // fire position 2
                        // fire position 1
                        // fire position 3
                        break;
                    case 3: // PPG intake
                        // RAPID FIRE AT 3
                        break;
                    case 0:
                        // rapid fire
                }
            case 22: // PGP motif
                switch (intake) {
                    case 1: //GPP intake
                        // fire position 3
                        // fire position 1
                        // fire position 2
                        break;
                    case 2: // PGP intake
                        // RAPID FIRE AT 3
                        break;
                    case 3: // PPG intake
                        // fire position 2
                        // fire position 3
                        // fire position 1
                        break;
                    case 0:
                        // rapid fire
                }
                break;
            case 23: // PPG motif
                switch (intake) {
                    case 1: //GPP intake
                        // RAPID FIRE AT 3
                        break;
                    case 2: // PGP intake
                        // fire position 3
                        // fire position 1
                        // fire position 2
                        break;
                    case 3: // PPG intake
                        // fire position 1
                        // fire position 3
                        // fire position 2
                        break;
                    case 0:
                        // rapid fire
                }
                break;
            case 0: // undetected motif
                // something idk yet
                // turn the LL to the motif via odometry?
                // rapid fire whatever?
        }
    }
}
