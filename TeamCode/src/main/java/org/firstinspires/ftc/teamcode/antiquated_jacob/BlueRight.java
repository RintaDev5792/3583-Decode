package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name= "BLUE-RIGHT")

public class BlueRight extends Skely {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        waitForStart();
        zeroClaws();
        runtime.reset();
        runMHAuto(-1,2);


    }
    public void zeroClaws() {
        clawLeft.setPosition(0);
        clawRight.setPosition(1);
        clawRotate.setPosition(0.25);


    }
}
