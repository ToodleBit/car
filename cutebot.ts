/**
 * Based on the code by ELECFREAKS
 */
//% color=#008C8C weight=10 icon="\uf1b9"
namespace ToodleCar {
const STM8_ADDRESSS = 0x10
    let _initEvents = true
	/**
	* Unit of Ultrasound Module
	*/
    export enum SonarUnit {
        //% block="cm"
        Centimeters,
        //% block="inches"
        Inches
    }
	/**
	* Select the motor on the left or right
	*/
    export enum MotorsList {
        //% blockId="M1" block="M1"
        M1 = 0,
        //% blockId="M2" block="M2"
        M2 = 1
    }
	/**
	* Select the servo on the S1 or S2
	*/
    export enum ServoList {
        //% block="S1"
        S1 = 0,
        //% block="S2"
        S2 = 1
    }
	/**
	* Select the RGBLights on the left or right
	*/
    export enum RGBLights {
        //% blockId="Right_light" block="Right_light"
        RGB_L = 1,
        //% blockId="Left_light" block="Left_light"
        RGB_R = 0,
        //% blockId="All" block="All"
        ALL = 3
    }
	/**
	* Status List of Tracking Modules
	*/
    export enum TrackingState {
        //% block="● ●" enumval=0
        L_R_line,

        //% block="◌ ●" enumval=1
        L_unline_R_line,

        //% block="● ◌" enumval=2
        L_line_R_unline,

        //% block="◌ ◌" enumval=3
        L_R_unline
    }
    export enum Direction {
        //% block="Forward" enumval=0
        forward,
        //% block="Backward" enumval=1
        backward,
        //% block="Left" enumval=2
        left,
        //% block="Right" enumval=3
        right
    }
    /**
    * Line Sensor events    MICROBIT_PIN_EVT_RISE
    */
    export enum MbEvents {
        //% block="Found" 
        FindLine = DAL.MICROBIT_PIN_EVT_FALL,
        //% block="Lost" 
        LoseLine = DAL.MICROBIT_PIN_EVT_RISE
    }
    /**
     * Pins used to generate events
     */
    export enum MbPins {
        //% block="Left" 
        Left = DAL.MICROBIT_ID_IO_P13,
        //% block="Right" 
        Right = DAL.MICROBIT_ID_IO_P14
    }
    /**
     * TODO: Set the speed of left and right wheels. 
     * @param lspeed Left wheel speed , eg: 50
     * @param rspeed Right wheel speed, eg: 50
     */
    //% blockId=MotorRun block="drive: left wheel %lspeed\\% |right wheel %rspeed\\%"
    //% lspeed.min=-100 lspeed.max=100
    //% rspeed.min=-100 rspeed.max=100
    export function motors(lspeed: number = 50, rspeed: number = 50): void {
        let buf = pins.createBuffer(4);
        if (lspeed > 100) {
            lspeed = 100;
        } else if (lspeed < -100) {
            lspeed = -100;
        }
        if (rspeed > 100) {
            rspeed = 100;
        } else if (rspeed < -100) {
            rspeed = -100;
        }
        if (lspeed > 0) {
            buf[0] = 0x01;    //左右轮 0x01左轮  0x02右轮
            buf[1] = 0x02;		//正反转0x02前进  0x01后退
            buf[2] = lspeed;	//速度
            buf[3] = 0;			//补位
            pins.i2cWriteBuffer(STM8_ADDRESSS, buf);  //写入左轮
        }
        else {
            buf[0] = 0x01;
            buf[1] = 0x01;
            buf[2] = lspeed * -1;
            buf[3] = 0;			//补位
            pins.i2cWriteBuffer(STM8_ADDRESSS, buf);  //写入左轮
        }
        if (rspeed > 0) {
            buf[0] = 0x02;
            buf[1] = 0x02;
            buf[2] = rspeed;
            buf[3] = 0;			//补位
            pins.i2cWriteBuffer(STM8_ADDRESSS, buf);  //写入左轮
        }
        else {
            buf[0] = 0x02;
            buf[1] = 0x01;
            buf[2] = rspeed * -1;
            buf[3] = 0;			//补位
            pins.i2cWriteBuffer(STM8_ADDRESSS, buf);  //写入左轮
        }

    }
    /**
    * TODO: Full speed operation lasts for 10 seconds,speed is 100.
    * @param dir Driving direction, eg: Direction.forward
    * @param speed Running speed, eg: 50
    * @param time Travel time, eg: 3
    */
    //% blockId=cutebot_move_time block="go %dir at %speed\\% for %time seconds"
   //% speed.min=15 speed.max=100
    export function moveTime(dir: Direction, speed: number, time: number): void {
        if (dir == 0) {
            motors(speed, speed);
            basic.pause(time * 1000)
            motors(0, 0)
        }
        if (dir == 1) {
            motors(-speed, -speed);
            basic.pause(time * 1000)
            motors(0, 0)
        }
        if (dir == 2) {
            motors(-speed, speed);
            basic.pause(time * 1000)
            motors(0, 0)
        }
        if (dir == 3) {
            motors(speed, -speed);
            basic.pause(time * 1000)
            motors(0, 0)
        }
    }

	/**
    * TODO: stopcar
    */
    //% blockId=cutebot_stopcar block="brake"
    export function stopcar(): void {
        motors(0, 0)
    }
    /**
    * TODO: Set LED headlights.
    */
    //% block="headlights %light color $color"
    //% color.shadow="colorNumberPicker"
    export function colorLight(light: RGBLights, color: number) {
        let r, g, b: number = 0
        r = color >> 16
        g = (color >> 8) & 0xFF
        b = color & 0xFF
        singleheadlights(light, r, g, b)
    }
	/**
	* TODO: Select a headlights and set the RGB color.
	* @param R R color value of RGB color, eg: 0
	* @param G G color value of RGB color, eg: 128
	* @param B B color value of RGB color, eg: 255
	*/
    //% inlineInputMode=inline
    //% blockId=RGB block="headlights %light color R:%r G:%g B:%b"
    //% r.min=0 r.max=255
    //% g.min=0 g.max=255
    //% b.min=0 b.max=255
	//% advanced=true
    export function singleheadlights(light: RGBLights, r: number, g: number, b: number): void {
        let buf = pins.createBuffer(4);
        if (light == 3) {
            buf[0] = 0x04;
            buf[1] = r;
            buf[2] = g;
            buf[3] = b;
            pins.i2cWriteBuffer(STM8_ADDRESSS, buf);
            buf[0] = 0x08;
            pins.i2cWriteBuffer(STM8_ADDRESSS, buf);
        }
        else {
            if (light == 0) {
                buf[0] = 0x04;
            }
            if (light == 1) {
                buf[0] = 0x08;
            }
            buf[1] = r;
            buf[2] = g;
            buf[3] = b;
            pins.i2cWriteBuffer(STM8_ADDRESSS, buf);
        }

    }
    /**
    * Close all headlights.
    */
    //% inlineInputMode=inline
    //% block="turn off headlights"
    export function closeheadlights(): void {
        let buf = pins.createBuffer(4);
        buf[0] = 0x04;
        buf[1] = 0;
        buf[2] = 0;
        buf[3] = 0;
        pins.i2cWriteBuffer(STM8_ADDRESSS, buf);
        buf[0] = 0x08;
        pins.i2cWriteBuffer(STM8_ADDRESSS, buf);
    }

    /**
	* Judging the Current Status of Tracking Module. 
	* @param state Four states of tracking module, eg: TrackingState.L_R_line
    */
    //% blockId=ringbitcar_tracking block="tracking state is %state"
	//% advanced=true
    export function tracking(state: TrackingState): boolean {

        pins.setPull(DigitalPin.P13, PinPullMode.PullNone)
        pins.setPull(DigitalPin.P14, PinPullMode.PullNone)
        let left_tracking = pins.digitalReadPin(DigitalPin.P13);
        let right_tracking = pins.digitalReadPin(DigitalPin.P14);
        if (left_tracking == 0 && right_tracking == 0 && state == 0) {
            return true;
        }
        else if (left_tracking == 1 && right_tracking == 0 && state == 1) {
            return true;
        }
        else if (left_tracking == 0 && right_tracking == 1 && state == 2) {
            return true;
        }
        else if (left_tracking == 1 && right_tracking == 1 && state == 3) {
            return true;
        }
        else {
            return false;
        }
    }
    /**
    * TODO: track one side
    * @param side Line sensor edge , eg: MbPins.Left
    * @param state Line sensor status, eg: MbEvents.FindLine
    */
    //% block="%side line sensor %state"
    //% state.fieldEditor="gridpicker" state.fieldOptions.columns=2
    //% side.fieldEditor="gridpicker" side.fieldOptions.columns=2
    //% advanced=true
    export function trackSide(side: MbPins, state: MbEvents): boolean {
        pins.setPull(DigitalPin.P13, PinPullMode.PullNone)
        pins.setPull(DigitalPin.P14, PinPullMode.PullNone)
        let left_tracking = pins.digitalReadPin(DigitalPin.P13);
        let right_tracking = pins.digitalReadPin(DigitalPin.P14);
        if (side == 0 && state == 1 && left_tracking == 1) {
            return true;
        }
        else if (side == 0 && state == 0 && left_tracking == 0) {
            return true;
        }
        else if (side == 1 && state == 1 && right_tracking == 1) {
            return true;
        }
        else if (side == 1 && state == 0 && right_tracking == 0) {
            return true;
        }
        else {
            return false;
        }
    }
    /**
    * TODO: Runs when line sensor finds or loses.
    */
    //% block="On %sensor| line %event"
    //% sensor.fieldEditor="gridpicker" sensor.fieldOptions.columns=2
    //% event.fieldEditor="gridpicker" event.fieldOptions.columns=2
    //% advanced=true
    export function trackEvent(sensor: MbPins, event: MbEvents, handler: Action) {
        initEvents();
        control.onEvent(<number>sensor, <number>event, handler);
        console.logValue("sensor", sensor)
        console.logValue("event", event)
    }
	/**
	* Cars can extend the ultrasonic function to prevent collisions and other functions.. 
	* @param Sonarunit two states of ultrasonic module, eg: Centimeters
    */
    //% blockId=ultrasonic block="sonar unit %unit"
    //% advanced=true
    export function ultrasonic(unit: SonarUnit, maxCmDistance = 500): number {
        // send pulse
        pins.setPull(DigitalPin.P8, PinPullMode.PullNone);
        pins.digitalWritePin(DigitalPin.P8, 0);
        control.waitMicros(2);
        pins.digitalWritePin(DigitalPin.P8, 1);
        control.waitMicros(10);
        pins.digitalWritePin(DigitalPin.P8, 0);

        // read pulse
        const d = pins.pulseIn(DigitalPin.P12, PulseValue.High, maxCmDistance * 50);

        switch (unit) {
            case SonarUnit.Centimeters:
                return Math.floor(d * 9 / 6 / 58);
            case SonarUnit.Inches:
                return Math.floor(d * 9 / 6 / 148);
            default:
                return d;
        }
    }

    function initEvents(): void {
        if (_initEvents) {
            pins.setEvents(DigitalPin.P13, PinEventType.Edge);
            pins.setEvents(DigitalPin.P14, PinEventType.Edge);
            _initEvents = false;
        }
    }
  //% shim=IR::init
  export function init(pin: Pins): void {
    return
  }

  /**
  * button pushed.
  */
  //% blockId=ir_received_event
  //% block="on |%btn| button pressed"
  //% shim=IR::onPressEvent
  //% advanced=true
  export function onPressEvent(btn: RemoteButton, body:Action): void {
    return
  }

   /**
     * Pause for the specified time in seconds
     * @param ms how long to pause for, eg: 1, 2, 5
     */
    //% help=basic/pause weight=54
    //% async block="pause (seconds) %pause" blockGap=16
    //% blockId=device_pause icon="\uf110"
    //% pause.shadow=timePicker
    void pause(int ms) {
      fiber_sleep(ms*1000);
    }
}
