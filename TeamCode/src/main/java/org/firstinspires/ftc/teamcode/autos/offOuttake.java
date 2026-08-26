package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardwareClasses.offSeasonIntake;

@Autonomous(name = "Outtake Run 5s", group = "Autonomous")
public class offOuttake extends LinearOpMode {

    private offSeasonIntake intake;

    @Override
    public void runOpMode() {
        intake = new offSeasonIntake(hardwareMap);

        waitForStart();

        intake.in();//using intake hardware class (ik in sounds weird but js look at the outtake cad and ull understand)
        sleep(5000);
        intake.stop();
    }
}