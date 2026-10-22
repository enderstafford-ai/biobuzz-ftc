# BioBuzz 38196 robot code

Our code lives in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`:

| File | What it is |
|------|------------|
| `MecanumDrive.java` | TeleOp "Mecanum Drive". Left stick = drive + slide, right stick = spin, D-pad = slow nudge, X = speed (Normal / Fast / Slow). |
| `WheelTest.java` | TeleOp "Wheel Test". Spin one wheel at a time; press B to flip a backward wheel (saved on the robot). |
| `DriveWheels.java` | The four drive motors and their directions, shared by both. |

Robot configuration (Control Hub motor ports): `frontLeft` 0, `backLeft` 1, `frontRight` 2, `backRight` 3, all goBILDA 5202/3/4.

Everything else is the official FTC SDK (v12.0). `upstream` remote = FIRST-Tech-Challenge/FtcRobotController, so SDK updates can be merged in.

## Pinpoint (odometry pods + gyro)
- Pinpoint: X port = forward/back pod, Y port = sideways pod, I2C port → Control Hub **I2C port 0**.
- Configuration: Control Hub → I2C Bus 0 → `goBILDA Pinpoint Odometry Computer`, name `pinpoint`.
- `HeadingSensor` asks the Pinpoint once at INIT and drops it if it doesn't answer. Polling a missing
  I2C device every loop jammed the hub's motor chip (motors dead, battery 0.00 V, "Control Hub not responding").
