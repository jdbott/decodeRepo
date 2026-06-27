package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardwareClasses.offSeasonIntake;

@Autonomous(name = "Intake Run 5s", group = "Autonomous")//runs for 5 seconds.
public class basicIntakeAuto extends LinearOpMode {

    private offSeasonIntake intake;

    @Override
    public void runOpMode() {
        intake = new offSeasonIntake(hardwareMap);

        waitForStart();

        intake.in();
        sleep(5000);
        intake.stop();
    }
}