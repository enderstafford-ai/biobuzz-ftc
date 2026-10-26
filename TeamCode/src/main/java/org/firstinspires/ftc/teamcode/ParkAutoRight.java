package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/* Leave + Park with the Loading Zone to the robot's right (see ParkAuto). Then Mecanum Drive is queued up for TELEOP. */
@Autonomous(name = "Auto: Zone on my RIGHT", group = "BioBuzz", preselectTeleOp = "Mecanum Drive")
public class ParkAutoRight extends ParkAuto {
    @Override
    protected int zoneSide() {
        return 1;
    }
}
