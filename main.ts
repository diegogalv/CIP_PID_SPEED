class PIDController {
    kp: number
    ki: number
    kd: number
    lastErr: number
    cumulErr: number
    constructor(kp: number, ki: number, kd: number) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.lastErr = 0;
        this.cumulErr = 0;
    }

    calcError(setPoint: number, current: number): number {
        return setPoint - current;
    }

    compute(setPoint: number, current: number, detaT: number): number {
        /* Compute all the working error variables */
        let error = this.calcError(setPoint, current);
        let dErr = (detaT == 0) ? 0 : this.calcError(error, this.lastErr) / detaT;

        /* Remember some variables for next time */
        this.lastErr = error;
        this.cumulErr += error * detaT;

        /* Compute PID Output */
        return this.kp * error + this.ki * this.cumulErr + this.kd * dErr;
    }

    reset() {
        this.lastErr = 0;
        this.cumulErr = 0;
    }
}
class PIDAngleController extends PIDController {
    constructor(kp: number, ki: number, kd: number) {
        super(kp, ki, kd);
    }

    calcError(setPoint: number, current: number) {
        let distance = (setPoint - current) % 360.0;
        if (distance < -180)
            return distance + 360;
        else if (distance > 179)
            return distance - 360;
        return distance;
    }
}
let control_states = 0
let rollingAvg = 0
let read_index = 0
let total = 0
let yawAngle = 0
let yawAngularSpeed = 0
const MAX_SPEED = 25
const kp = 0.0005
const ki = 0
const kd = 0.019
let pid_speed = new PIDController(kp, ki, kd)
let pid_attitude = new PIDAngleController(1.25, 0, 200)
let motorSpeed = 0;
let timecur: number;
let timeprev;
let timestart;
let num_read = 5
let giro = 0
let readings = [
    0,
    0,
    0,
    0,
    0
]
const giro_err = 44.0880282
let counts = 0
let thorttle = 76
let targetAtt: number
timecur = control.millis()
timestart = timecur

/**
 * Control PID
 */
//% color="#03AA74" weight=88 icon="\uf021" blockGap=8
//% groups='Positional'
namespace PID {
    //% fixedInstances


/**
    * Inicializa en PID
    */
//% blockId="PID_control_inicializado"
//% block="control_init"
export function Init(): void{
    radio.setTransmitPower(7)
    SENMPU6050.initMPU6050()
    pins.analogSetPeriod(AnalogPin.P0, 21)
}

   


    /**
             * Set the possible rotation range angles for the servo between 0 and 180
             * @param minAngle the minimum angle from 0 to 90
             * @param maxAngle the maximum angle from 90 to 180
             */
        //% help=PID/PID_Control
        //% blockId=pidcontrol block="PID modo %control_mode en el angulo %angle"
        //% control_mode.min=0 control_mode.max=2
        //% angle.min=0 maxAngle.max=360 angle.defl=180
        //% group="Positional"
        //% blockGap=8
    export function Control_PID(control_mode :number, angle:number): void {
    
    if (control.millis() - timecur > 10) {
        timeprev = timecur
        timecur = control.millis()
        // Measure Gyro value
        yawAngularSpeed = ((SENMPU6050.gyroscope(axisXYZ.z, gyroSen.range_250_dps) * 57.295) - giro_err) / 65.5
        yawAngle += (yawAngularSpeed * (timecur - timeprev) / 1000)
        // Put angle between -180 and 180
        while (yawAngle <= -180) {
            yawAngle += 360
        }
        while (yawAngle > 180) {
            yawAngle += - 360
        }
        // Low Pass Filter the angular speed (https://www.arduino.cc/en/Tutorial/BuiltInExamples/Smoothing)
        total = total - readings[read_index]
        readings[read_index] = yawAngularSpeed
        total = total + readings[read_index]
        read_index = read_index + 1
        if (read_index >= num_read) {
            read_index = 0
        }
        rollingAvg = total / num_read
        //serial.writeValue('z' , rollingAvg)

        if (control_mode == 0) {
            // Detumbling only
            motorSpeed += pid_speed.compute(0, rollingAvg, timecur - timeprev);
        }
        else {
            // Change set point
            if (control_mode == 2) {
                //    counts++;
                //    if (counts == 300) {
                //        counts = 0;
                //        if (targetAttitude == 0)
                targetAtt = angle;
                //        else
                //            targetAttitude = 0;
                //    }
            }
            if (control_states == 1 && Math.abs(rollingAvg) > 400 /* °/s */) {
                control_states = 0;
            }
            if (control_states == 0 && Math.abs(rollingAvg) < 50 /* °/s */)
                control_states = 1;

            //FSM action
            if (control_states == 0) {
                motorSpeed += pid_speed.compute(0, rollingAvg, timecur - timeprev);
                pid_attitude.compute(angle, yawAngle, timecur - timeprev);
            } else {
                motorSpeed += pid_speed.compute(pid_attitude.compute(angle, yawAngle, timecur - timeprev), rollingAvg, timecur - timeprev);
            }
        }
        if (motorSpeed > MAX_SPEED) motorSpeed = MAX_SPEED;
        else if (motorSpeed < -MAX_SPEED) motorSpeed = -MAX_SPEED;
        radio.sendNumber(yawAngle)
        giro = thorttle - motorSpeed;
        pins.analogWritePin(AnalogPin.P0, giro);
    }
}
}