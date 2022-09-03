enum PingUnit {
    //% block="微秒"
    MicroSeconds,
    //% block="厘米"
    Centimeters,
    //% block="英寸"
    Inches
}
enum PINs {
    P0,
    P1,
    P2,
    P3,
    P4,
    P5,
    P6,
    P7,
    P8,
    P9,
    P10,
    P11,
    P12,
    P13,
    P14,
    P15,
    P16,
    P19,
    P20
}
enum TMP36Type {
    //% block="(℃)" enumval=0
    TMP36_temperature_C,

    //% block="(℉)" enumval=1
    TMP36_temperature_F,
}
enum RGB {
    //% block="红"
    RED,
    //% block="绿"
    GREEN,
    //% block="蓝"
    BLUE,
    //% block="全部"
    CLEAR
}
enum NeoPixelColors {
    //% block=红
    Red = 0xFF0000,
    //% block=橙
    Orange = 0xFFA500,
    //% block=黄
    Yellow = 0xFFFF00,
    //% block=绿
    Green = 0x00FF00,
    //% block=蓝
    Blue = 0x0000FF,
    //% block=靛蓝
    Indigo = 0x4b0082,
    //% block=紫罗兰
    Violet = 0x8a2be2,
    //% block=紫
    Purple = 0xFF00FF,
    //% block=白
    White = 0xFFFFFF,
    //% block=黑
    Black = 0x000000
}
enum NeoPixelMode {
    //% block="RGB (GRB 格式)"
    RGB = 1,
    //% block="RGB+W"
    RGBW = 2,
    //% block="RGB (RGB 格式)"
    RGB_RGB = 3
}
namespace Microbit {

    export enum DHT11_state {
        //% block="温度(℃)" enumval=0
        DHT11_temperature_C,

        //% block="湿度(0~100)" enumval=1
        DHT11_humidity,
    }

    export enum Distance_Unit_List {
        //% block="厘米" 
        Distance_Unit_cm,

        //% block="英寸"
        Distance_Unit_inch,
    }






    // Functions for reading Particle from the gatorParticle in Particle or straight adv value
	
	/**
	* Initializes the gator:particle sensor, must be called on power up
	*/	
	//% weight=30 
	//% blockId="gatorParticle_begin" 
	//% block="initialize gator:Particle sensor"
	//% shim=Microbit::begin
	export function begin(){
		return
	}
		
	/**
	* Reads either the Red or Infrared detection channels
	*/
	//% weight=29 
	//% blockId="gatorParticle_color" 
	//% block="get %LEDToRead value"
	//% shim=Microbit::color
	export function color(type: LEDToRead): number{
		return 0
	}
	
	/**
	* Set which LED's we want the sensor to update and read.
	*/	
	//% weight=28
	//% blockId="gatorParticle_setReadMode"
	//% block="set LED mode to read %LEDMode"
	//% shim=Microbit::setReadMode
	//% advanced=true
	export function setReadMode(mode: LEDMode)
	{
		return
	}

	/**
	* Set the amplitude of either Red or Infrared LED
	*/	
	//% weight=27
	//% blockId="gatorParticle_setAmplitude"
	//% block="change strength of %LEDToRead | to %myBrightness"
	//% shim=Microbit::setAmplitude
	//% advanced=true
	export function setAmplitude(led: LEDToRead, myBrightness: number)
	{
		return
	}
	
	/**
	* Grab the heartbeat from the sensor in either beats per minute, or an average of the last 4 BPM readings.
	*/
	//% weight=26
	//% blockId="gatorParticle_heartbeat"
	//% block="detect heartbeat in %HeartbeatType"
	//% shim=Microbit::heartbeat
	export function heartbeat(type: HeartbeatType): number
	{
		return 0
	}







    //% blockId="readsoilmoisture" block="土壤湿度传感器 %soilhumiditypin"
    //% subcategory=传感器
    export function ReadSoilHumidity(soilmoisturepin: AnalogPin): number {
        let voltage = 0;
        let soilmoisture = 0;
        voltage = pins.map(
            pins.analogReadPin(soilmoisturepin),
            0,
            1023,
            0,
            1023
        );
        soilmoisture = voltage;
        return Math.round(soilmoisture);
    }
    //% blockId="readlightintensity" block="光敏传感器 %lightintensitypin"
    //% subcategory=传感器
    export function ReadLightIntensity(lightintensitypin: AnalogPin): number {
        let voltage2 = 0;
        let lightintensity = 0;
        voltage2 = pins.map(
            pins.analogReadPin(lightintensitypin),
            0,
            1023,
            0,
            1023
        );
        lightintensity = voltage2;
        return Math.round(1023 - lightintensity);
    }
    /**
    * toggle fans
    */
    //% blockId=fans block="电机风扇 %pin 切换至 $fanstate || 速度 %speed"
    //% fanstate.shadow="toggleOnOff"
    //% speed.min=0 speed.max=1023
    //% expandableArgumentMode="toggle"
    //% subcategory=执行器
    export function motorFan(pin: AnalogPin, fanstate: boolean, speed: number = 1023): void {
        if (fanstate) {
            pins.analogSetPeriod(pin, 1023)
            pins.analogWritePin(pin, Math.map(speed, 0, 1023, 0, 1023))
        }
        else {
            pins.analogWritePin(pin, 0)
            speed = 0
        }
    }

    /**
    * get Ultrasonic distance
    */
    //% blockId=sonarbit
    //% distance_unit.fieldEditor="gridpicker"
    //% distance_unit.fieldOptions.columns=2
    //% subcategory=传感器
    //% blockId=sonar_ping block="超声波传感器 echo %echo trig %trig 单位 %unit"
    export function ping(trig: DigitalPin, echo: DigitalPin, unit: PingUnit, maxCmDistance = 500): number {
        // send pulse
        pins.setPull(trig, PinPullMode.PullNone);
        pins.digitalWritePin(trig, 0);
        control.waitMicros(2);
        pins.digitalWritePin(trig, 1);
        control.waitMicros(10);
        pins.digitalWritePin(trig, 0);

        // read pulse
        const d = pins.pulseIn(echo, PulseValue.High, maxCmDistance * 58);

        switch (unit) {
            case PingUnit.Centimeters: return Math.idiv(d, 58);
            case PingUnit.Inches: return Math.idiv(d, 148);
            default: return d;
        }
    }

    /**
    * toggle led
    */
    //% blockId=LEDR block="红色 LED %pin 切换到 $ledstate || 亮度 %brightness"
    //% brightness.min=0 brightness.max=1023
    //% ledstate.shadow="toggleOnOff"
    //% expandableArgumentMode="toggle"
    //% subcategory=执行器
    export function ledRBrightness(pin: AnalogPin, ledstate: boolean, brightness: number = 1023): void {
        if (ledstate) {
            pins.analogSetPeriod(pin, 1023)
            pins.analogWritePin(pin, Math.map(brightness, 1023, 0, 0, 1023))
        }
        else {
            pins.analogWritePin(pin, 1023)
            brightness = 1023
        }
    }

    /**
    * toggle led
    */
    //% blockId=LEDG block="绿色 LED %pin 切换到 $ledstate || 亮度 %brightness"
    //% brightness.min=0 brightness.max=1023
    //% ledstate.shadow="toggleOnOff"
    //% expandableArgumentMode="toggle"
    //% subcategory=执行器
    export function ledGBrightness(pin: AnalogPin, ledstate: boolean, brightness: number = 1023): void {
        if (ledstate) {
            pins.analogSetPeriod(pin, 1023)
            pins.analogWritePin(pin, Math.map(brightness, 1023, 0, 0, 1023))
        }
        else {
            pins.analogWritePin(pin, 1023)
            brightness = 1023
        }
    }

    /**
    * toggle led
    */
    //% blockId=LEDB block="蓝色 LED %pin 切换到 $ledstate || 亮度 %brightness"
    //% brightness.min=0 brightness.max=1023
    //% ledstate.shadow="toggleOnOff"
    //% expandableArgumentMode="toggle"
    //% subcategory=执行器
    export function ledBBrightness(pin: AnalogPin, ledstate: boolean, brightness: number = 1023): void {
        if (ledstate) {
            pins.analogSetPeriod(pin, 1023)
            pins.analogWritePin(pin, Math.map(brightness, 1023, 0, 0, 1023))
        }
        else {
            pins.analogWritePin(pin, 1023)
            brightness = 1023
        }
    }

    /**
    * toggle led
    */
    //% blockId=LEDY block="黄色 LED %pin 切换到 $ledstate || 亮度 %brightness"
    //% brightness.min=0 brightness.max=1023
    //% ledstate.shadow="toggleOnOff"
    //% expandableArgumentMode="toggle"
    //% subcategory=执行器
    export function ledYBrightness(pin: AnalogPin, ledstate: boolean, brightness: number = 1023): void {
        if (ledstate) {
            pins.analogSetPeriod(pin, 1023)
            pins.analogWritePin(pin, Math.map(brightness, 1023, 0, 0, 1023))
        }
        else {
            pins.analogWritePin(pin, 1023)
            brightness = 1023
        }
    }

    const PCA9685_ADD = 0x40
    const MODE1 = 0x00
    const LED0_ON_L = 0x06
    const PRESCALE = 0xFE

    let initialized = false

    export enum enPos {
        //% blockId="forward" block="前进"
        forward = 1,
        //% blockId="stop" block="后退"
        stop = 2
    }

    export enum enServo {
        S1 = 0,
        S2,
        S3,
        S4
    }

    export enum enMotors {
        M1 = 8,
        M2 = 10,
        M3 = 12,
        M4 = 14
    }

    function i2cwrite(addr: number, reg: number, value: number) {
        let buf = pins.createBuffer(2)
        buf[0] = reg
        buf[1] = value
        pins.i2cWriteBuffer(addr, buf)
    }

    function i2cread(addr: number, reg: number) {
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE);
        let val = pins.i2cReadNumber(addr, NumberFormat.UInt8BE);
        return val;
    }

    function initPCA9685(): void {
        i2cwrite(PCA9685_ADD, MODE1, 0x00)
        setFreq(50);
    }

    function setFreq(freq: number): void {
        // Constrain the frequency
        let prescaleval = 25000000;
        prescaleval /= 4096;
        prescaleval /= freq;
        prescaleval -= 1;
        let prescale = prescaleval; //Math.Floor(prescaleval + 0.5);
        let oldmode = i2cread(PCA9685_ADD, MODE1);
        let newmode = (oldmode & 0x7F) | 0x10; // sleep
        i2cwrite(PCA9685_ADD, MODE1, newmode); // go to sleep
        i2cwrite(PCA9685_ADD, PRESCALE, prescale); // set the prescaler
        i2cwrite(PCA9685_ADD, MODE1, oldmode);
        control.waitMicros(5000);
        i2cwrite(PCA9685_ADD, MODE1, oldmode | 0xa1);
    }

    function setPwm(channel: number, on: number, off: number): void {
        if (channel < 0 || channel > 15)
            return;
        if (!initialized) {
            initPCA9685();
        }
        let buf2 = pins.createBuffer(5);
        buf2[0] = LED0_ON_L + 4 * channel;
        buf2[1] = on & 0xff;
        buf2[2] = (on >> 8) & 0xff;
        buf2[3] = off & 0xff;
        buf2[4] = (off >> 8) & 0xff;
        pins.i2cWriteBuffer(PCA9685_ADD, buf2);
    }

    //% blockId=SuperBit_Servo4 block="Geek舵机| %num|角度 %value"
    //% num.min=1 num.max=4 value.min=0 value.max=300
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=20
    //% subcategory=执行器
    export function Servo4(num: enServo, value: number): void {

        // 50hz: 20,000 us
        let us = (value * 1800 * 0.6 / 180 + 600); // 0.6 ~ 2.4
        let pwm = us * 4096 / 20000;
        setPwm(num, 0, pwm);

    }

    //% blockId=SuperBit_MotorRun block="电机|%index|速度(-255~255) %speed"
    //% speed.min=-255 speed.max=255
    //% subcategory=执行器
    export function MotorRun(index: enMotors, speed: number): void {
        if (!initialized) {
            initPCA9685()
        }
        speed = speed * 16; // map 255 to 4096
        if (speed >= 4096) {
            speed = 4095
        }
        if (speed <= -4096) {
            speed = -4095
        }

        let a = index
        let b = index + 1

        if (a > 10) {
            if (speed >= 0) {
                setPwm(a, 0, speed)
                setPwm(b, 0, 0)
            } else {
                setPwm(a, 0, 0)
                setPwm(b, 0, -speed)
            }
        }
        else {
            if (speed >= 0) {
                setPwm(b, 0, speed)
                setPwm(a, 0, 0)
            } else {
                setPwm(b, 0, 0)
                setPwm(a, 0, -speed)
            }
        }
    }

    //% blockId="elecmagnet" block="电磁铁 %pin 切换至 %magState || 磁力 %force"
    //% magState.shadow="toggleOnOff"
    //% force.min=0 force.max=1023
    //% expandableArgumentMode="toggle"
    //% subcategory=执行器
    export function elecmagnet(pin: AnalogPin, magState: boolean, force: number = 1023): void {

        if (magState) {
            pins.analogSetPeriod(pin, 1023)
            pins.analogWritePin(pin, Math.map(force, 0, 1023, 0, 1023))
        }
        else {
            pins.analogWritePin(pin, 0)
            force = 0
        }
    }
    //% blockId="pinpullup" block="巡线传感器 %pin 引脚为上拉"
    //% subcategory=执行器
    export function pinpullup(pin: PINs): void {
        switch (pin) {
            case PINs.P0: pins.setPull(DigitalPin.P0, PinPullMode.PullUp);
            case PINs.P1: pins.setPull(DigitalPin.P1, PinPullMode.PullUp);
            case PINs.P2: pins.setPull(DigitalPin.P2, PinPullMode.PullUp);
            case PINs.P3: pins.setPull(DigitalPin.P3, PinPullMode.PullUp);
            case PINs.P4: pins.setPull(DigitalPin.P4, PinPullMode.PullUp);
            case PINs.P5: pins.setPull(DigitalPin.P5, PinPullMode.PullUp);
            case PINs.P6: pins.setPull(DigitalPin.P6, PinPullMode.PullUp);
            case PINs.P7: pins.setPull(DigitalPin.P7, PinPullMode.PullUp);
            case PINs.P8: pins.setPull(DigitalPin.P8, PinPullMode.PullUp);
            case PINs.P9: pins.setPull(DigitalPin.P9, PinPullMode.PullUp);
            case PINs.P10: pins.setPull(DigitalPin.P10, PinPullMode.PullUp);
            case PINs.P11: pins.setPull(DigitalPin.P11, PinPullMode.PullUp);
            case PINs.P12: pins.setPull(DigitalPin.P12, PinPullMode.PullUp);
            case PINs.P13: pins.setPull(DigitalPin.P13, PinPullMode.PullUp);
            case PINs.P14: pins.setPull(DigitalPin.P14, PinPullMode.PullUp);
            case PINs.P15: pins.setPull(DigitalPin.P15, PinPullMode.PullUp);
            case PINs.P16: pins.setPull(DigitalPin.P16, PinPullMode.PullUp);
            case PINs.P19: pins.setPull(DigitalPin.P19, PinPullMode.PullUp);
            case PINs.P20: pins.setPull(DigitalPin.P20, PinPullMode.PullUp);
        }


    }

    //% blockId="laser_controller" block="激光 %pin 切换至 %laserState || 激光 %intensity"
    //% laserState.shadow="toggleOnOff"
    //% intensity.min=0 intensity.max=1023
    //% expandableArgumentMode="toggle"
    //% subcategory=执行器
    export function laserController(pin: AnalogPin, laserState: boolean, intensity: number = 1023): void {

        if (laserState) {
            pins.analogSetPeriod(pin, 1023)
            pins.analogWritePin(pin, Math.map(intensity, 0, 1023, 0, 1023))
        }
        else {
            pins.analogWritePin(pin, 0)
            intensity = 0
        }
    }

    //% blockId="octopus_ReadWaterLevel" block="水位传感器 %waterlevelpin"
    //% subcategory=传感器
    export function ReadWaterLevel(waterlevelpin: AnalogPin): number {
        let voltage4 = 0;
        let waterLevel = 0;
        voltage4 = pins.map(
            pins.analogReadPin(waterlevelpin),
            0,
            1023,
            0,
            1023
        );
        waterLevel = voltage4;
        return Math.round(waterLevel);
    }

    //% blockId="ReadGasConcentration" block="可燃气体传感器 %gasconcentrationpin"
    //% subcategory=传感器
    export function ReadGasConcentration(gasconcentrationpin: AnalogPin): number {
        let voltage5 = 0;
        let gasConcentration = 0;
        voltage5 = pins.map(
            pins.analogReadPin(gasconcentrationpin),
            0,
            1023,
            0,
            1023
        );
        gasConcentration = voltage5;
        return Math.round(gasConcentration);
    }

    //% blockId="Readflame" block="火焰传感器 %flamepin"
    //% subcategory=传感器
    export function Readflame(flamepin: AnalogPin): number {
        let voltage6 = 0;
        let flame = 0;
        voltage6 = pins.map(
            pins.analogReadPin(flamepin),
            0,
            1023,
            0,
            1023
        );
        flame = voltage6;
        return Math.round(flame);
    }

    //% blockId="ReadGrayLevel" block="灰度传感器 %graylevelpin"
    //% subcategory=传感器
    export function ReadGrayLevel(graylevelpin: AnalogPin): number {
        let voltage7 = 0;
        let grayLevel = 0;
        voltage7 = pins.map(
            pins.analogReadPin(graylevelpin),
            0,
            1023,
            80,
            1023
        );
        grayLevel = voltage7;
        return Math.round(grayLevel);
    }

    //% blockId="readWaterTemp" block="防水温度传感器 %waterproofpin"
    //% subcategory=传感器
    export function readWaterTemp(waterproofpin: AnalogPin): number {
        let voltage22 = 0;
        let waterProofTemp = 0;
        voltage22 = pins.analogReadPin(waterproofpin);//%获得原始值
        if (voltage22 > 1001) { waterProofTemp = 100; }
        else if (voltage22 > 1000) { waterProofTemp = 98; }
        else if (voltage22 > 999) { waterProofTemp = 97; }
        else if (voltage22 > 998) { waterProofTemp = 96; }
        else if (voltage22 > 997) { waterProofTemp = 95; }
        else if (voltage22 > 996) { waterProofTemp = 93; }
        else if (voltage22 > 995) { waterProofTemp = 92; }
        else if (voltage22 > 994) { waterProofTemp = 91; }
        else if (voltage22 > 993) { waterProofTemp = 90; }
        else if (voltage22 > 992) { waterProofTemp = 89; }
        else if (voltage22 > 991) { waterProofTemp = 88; }
        else if (voltage22 > 990) { waterProofTemp = 87; }
        else if (voltage22 > 989) { waterProofTemp = 86; }
        else if (voltage22 > 988) { waterProofTemp = 85; }
        else if (voltage22 > 987) { waterProofTemp = 84; }
        else if (voltage22 > 986) { waterProofTemp = 83; }
        else if (voltage22 > 985) { waterProofTemp = 82; }
        else if (voltage22 > 984) { waterProofTemp = 81; }
        else if (voltage22 > 982) { waterProofTemp = 80; }
        else if (voltage22 > 981) { waterProofTemp = 79; }
        else if (voltage22 > 980) { waterProofTemp = 78; }
        else if (voltage22 > 978) { waterProofTemp = 77; }
        else if (voltage22 > 977) { waterProofTemp = 76; }
        else if (voltage22 > 975) { waterProofTemp = 75; }
        else if (voltage22 > 974) { waterProofTemp = 74; }
        else if (voltage22 > 972) { waterProofTemp = 73; }
        else if (voltage22 > 971) { waterProofTemp = 72; }
        else if (voltage22 > 969) { waterProofTemp = 71; }
        else if (voltage22 > 967) { waterProofTemp = 70; }
        else if (voltage22 > 965) { waterProofTemp = 69; }
        else if (voltage22 > 963) { waterProofTemp = 68; }
        else if (voltage22 > 961) { waterProofTemp = 67; }
        else if (voltage22 > 959) { waterProofTemp = 66; }
        else if (voltage22 > 957) { waterProofTemp = 65; }
        else if (voltage22 > 955) { waterProofTemp = 64; }
        else if (voltage22 > 953) { waterProofTemp = 63; }
        else if (voltage22 > 950) { waterProofTemp = 62; }
        else if (voltage22 > 948) { waterProofTemp = 61; }
        else if (voltage22 > 943) { waterProofTemp = 59; }
        else if (voltage22 > 940) { waterProofTemp = 58; }
        else if (voltage22 > 937) { waterProofTemp = 57; }
        else if (voltage22 > 934) { waterProofTemp = 56; }
        else if (voltage22 > 931) { waterProofTemp = 55; }
        else if (voltage22 > 928) { waterProofTemp = 54; }
        else if (voltage22 > 924) { waterProofTemp = 53; }
        else if (voltage22 > 921) { waterProofTemp = 52; }
        else if (voltage22 > 917) { waterProofTemp = 51; }
        else if (voltage22 > 914) { waterProofTemp = 51; }
        else if (voltage22 > 910) { waterProofTemp = 49; }
        else if (voltage22 > 906) { waterProofTemp = 48; }
        else if (voltage22 > 902) { waterProofTemp = 47; }
        else if (voltage22 > 898) { waterProofTemp = 46; }
        else if (voltage22 > 893) { waterProofTemp = 45; }
        else if (voltage22 > 889) { waterProofTemp = 44; }
        else if (voltage22 > 884) { waterProofTemp = 43; }
        else if (voltage22 > 879) { waterProofTemp = 42; }
        else if (voltage22 > 874) { waterProofTemp = 41; }
        else if (voltage22 > 869) { waterProofTemp = 40; }
        else if (voltage22 > 864) { waterProofTemp = 39; }
        else if (voltage22 > 858) { waterProofTemp = 38; }
        else if (voltage22 > 852) { waterProofTemp = 37; }
        else if (voltage22 > 846) { waterProofTemp = 36; }
        else if (voltage22 > 840) { waterProofTemp = 35; }
        else if (voltage22 > 834) { waterProofTemp = 34; }
        else if (voltage22 > 827) { waterProofTemp = 33; }
        else if (voltage22 > 821) { waterProofTemp = 32; }
        else if (voltage22 > 814) { waterProofTemp = 31; }
        else if (voltage22 > 806) { waterProofTemp = 30; }
        else if (voltage22 > 799) { waterProofTemp = 29; }
        else if (voltage22 > 791) { waterProofTemp = 28; }
        else if (voltage22 > 784) { waterProofTemp = 27; }
        else if (voltage22 > 776) { waterProofTemp = 26; }
        else if (voltage22 > 767) { waterProofTemp = 25; }
        else if (voltage22 > 759) { waterProofTemp = 24; }
        else if (voltage22 > 750) { waterProofTemp = 23; }
        else if (voltage22 > 741) { waterProofTemp = 22; }
        else if (voltage22 > 732) { waterProofTemp = 21; }
        else if (voltage22 > 713) { waterProofTemp = 19; }
        else if (voltage22 > 703) { waterProofTemp = 18; }
        else if (voltage22 > 692) { waterProofTemp = 17; }
        else if (voltage22 > 682) { waterProofTemp = 16; }
        else if (voltage22 > 671) { waterProofTemp = 15; }
        else if (voltage22 > 661) { waterProofTemp = 14; }
        else if (voltage22 > 650) { waterProofTemp = 13; }
        else if (voltage22 > 638) { waterProofTemp = 12; }
        else if (voltage22 > 627) { waterProofTemp = 11; }
        else if (voltage22 > 615) { waterProofTemp = 10; }
        else if (voltage22 > 604) { waterProofTemp = 9; }
        else if (voltage22 > 592) { waterProofTemp = 8; }
        else if (voltage22 > 579) { waterProofTemp = 7; }
        else if (voltage22 > 567) { waterProofTemp = 6; }
        else if (voltage22 > 555) { waterProofTemp = 5; }
        else if (voltage22 > 542) { waterProofTemp = 4; }
        else if (voltage22 > 530) { waterProofTemp = 3; }
        else if (voltage22 > 517) { waterProofTemp = 2; }
        else if (voltage22 > 504) { waterProofTemp = 1; }
        else { waterProofTemp = 0; }
        return waterProofTemp;
    }

    //% blockId=potentiometerRead
    //% block="电位器 %pin"
    //% subcategory=传感器
    export function potentiometerRead(pin: AnalogPin): number {
        return pins.analogReadPin(pin);
    }

    //% blockId=buttonState
    //% block="按钮传感器 %pin"
    //% subcategory=传感器
    export function buttonState(pin: DigitalPin): number {

        return pins.digitalReadPin(pin);
    }
    //% blockId=followState
    //% block="巡线传感器 %pin"
    //% subcategory=传感器
    export function followState(pin: PINs): number {
        switch (pin) {
            case PINs.P0: return pins.digitalReadPin(DigitalPin.P0);
            case PINs.P1: return pins.digitalReadPin(DigitalPin.P1);
            case PINs.P2: return pins.digitalReadPin(DigitalPin.P2);
            case PINs.P3: return pins.digitalReadPin(DigitalPin.P3);
            case PINs.P4: return pins.digitalReadPin(DigitalPin.P4);
            case PINs.P5: return pins.digitalReadPin(DigitalPin.P5);
            case PINs.P6: return pins.digitalReadPin(DigitalPin.P6);
            case PINs.P7: return pins.digitalReadPin(DigitalPin.P7);
            case PINs.P8: return pins.digitalReadPin(DigitalPin.P8);
            case PINs.P9: return pins.digitalReadPin(DigitalPin.P9);
            case PINs.P10: return pins.digitalReadPin(DigitalPin.P10);
            case PINs.P11: return pins.digitalReadPin(DigitalPin.P11);
            case PINs.P12: return pins.digitalReadPin(DigitalPin.P12);
            case PINs.P13: return pins.digitalReadPin(DigitalPin.P13);
            case PINs.P14: return pins.digitalReadPin(DigitalPin.P14);
            case PINs.P15: return pins.digitalReadPin(DigitalPin.P15);
            case PINs.P16: return pins.digitalReadPin(DigitalPin.P16);
            case PINs.P19: return pins.digitalReadPin(DigitalPin.P19);
            case PINs.P20: return pins.digitalReadPin(DigitalPin.P20);

        }

    }

    //% blockId=farState
    //% block="远距离光电传感器 %pin"
    //% subcategory=传感器
    export function farState(pin: DigitalPin): number {
        return pins.digitalReadPin(pin);
    }

    //% blockId=hState
    //% block="霍尔传感器 %pin"
    //% subcategory=传感器
    export function hState(pin: DigitalPin): number {
        return pins.digitalReadPin(pin);
    }

    enum LCS_Constants {
        // Constants
        ADDRESS = 0x29,
        ID = 0x12, // Register should be equal to 0x44 for the TCS34721 or TCS34725, or 0x4D for the TCS34723 or TCS34727.

        COMMAND_BIT = 0x80,

        ENABLE = 0x00,
        ENABLE_AIEN = 0x10, // RGBC Interrupt Enable
        ENABLE_WEN = 0x08, // Wait enable - Writing 1 activates the wait timer
        ENABLE_AEN = 0x02, // RGBC Enable - Writing 1 actives the ADC, 0 disables it
        ENABLE_PON = 0x01, // Power on - Writing 1 activates the internal oscillator, 0 disables it
        ATIME = 0x01, // Integration time
        WTIME = 0x03, // Wait time (if ENABLE_WEN is asserted)
        AILTL = 0x04, // Clear channel lower interrupt threshold
        AILTH = 0x05,
        AIHTL = 0x06, // Clear channel upper interrupt threshold
        AIHTH = 0x07,
        PERS = 0x0C, // Persistence register - basic SW filtering mechanism for interrupts
        PERS_NONE = 0x00, // Every RGBC cycle generates an interrupt
        PERS_1_CYCLE = 0x01, // 1 clean channel value outside threshold range generates an interrupt
        PERS_2_CYCLE = 0x02, // 2 clean channel values outside threshold range generates an interrupt
        PERS_3_CYCLE = 0x03, // 3 clean channel values outside threshold range generates an interrupt
        PERS_5_CYCLE = 0x04, // 5 clean channel values outside threshold range generates an interrupt
        PERS_10_CYCLE = 0x05, // 10 clean channel values outside threshold range generates an interrupt
        PERS_15_CYCLE = 0x06, // 15 clean channel values outside threshold range generates an interrupt
        PERS_20_CYCLE = 0x07, // 20 clean channel values outside threshold range generates an interrupt
        PERS_25_CYCLE = 0x08, // 25 clean channel values outside threshold range generates an interrupt
        PERS_30_CYCLE = 0x09, // 30 clean channel values outside threshold range generates an interrupt
        PERS_35_CYCLE = 0x0A, // 35 clean channel values outside threshold range generates an interrupt
        PERS_40_CYCLE = 0x0B, // 40 clean channel values outside threshold range generates an interrupt
        PERS_45_CYCLE = 0x0C, // 45 clean channel values outside threshold range generates an interrupt
        PERS_50_CYCLE = 0x0D, // 50 clean channel values outside threshold range generates an interrupt
        PERS_55_CYCLE = 0x0E, // 55 clean channel values outside threshold range generates an interrupt
        PERS_60_CYCLE = 0x0F, // 60 clean channel values outside threshold range generates an interrupt
        CONFIG = 0x0D,
        CONFIG_WLONG = 0x02, // Choose between short and long (12x) wait times via WTIME
        CONTROL = 0x0F, // Set the gain level for the sensor
        STATUS = 0x13,
        STATUS_AINT = 0x10, // RGBC Clean channel interrupt
        STATUS_AVALID = 0x01, // Indicates that the RGBC channels have completed an integration cycle

        CDATAL = 0x14, // Clear channel data
        CDATAH = 0x15,
        RDATAL = 0x16, // Red channel data
        RDATAH = 0x17,
        GDATAL = 0x18, // Green channel data
        GDATAH = 0x19,
        BDATAL = 0x1A, // Blue channel data
        BDATAH = 0x1B,

        GAIN_1X = 0x00, //  1x gain
        GAIN_4X = 0x01, //  4x gain
        GAIN_16X = 0x02, // 16x gain
        GAIN_60X = 0x03  // 60x gain
    }

    let LCS_integration_time_val = 0

    // I2C functions

    function I2C_WriteReg8(addr: number, reg: number, val: number) {
        let buf3 = pins.createBuffer(2)
        buf3.setNumber(NumberFormat.UInt8BE, 0, reg)
        buf3.setNumber(NumberFormat.UInt8BE, 1, val)
        pins.i2cWriteBuffer(addr, buf3)
    }

    function I2C_ReadReg8(addr: number, reg: number): number {
        let buf4 = pins.createBuffer(1)
        buf4.setNumber(NumberFormat.UInt8BE, 0, reg)
        pins.i2cWriteBuffer(addr, buf4)
        buf4 = pins.i2cReadBuffer(addr, 1)
        return buf4.getNumber(NumberFormat.UInt8BE, 0);
    }

    function I2C_ReadReg16(addr: number, reg: number): number {
        let buf5 = pins.createBuffer(1)
        buf5.setNumber(NumberFormat.UInt8BE, 0, reg)
        pins.i2cWriteBuffer(addr, buf5)
        buf5 = pins.i2cReadBuffer(addr, 2)
        // Little endian
        return ((buf5.getNumber(NumberFormat.UInt8BE, 1) << 8) | buf5.getNumber(NumberFormat.UInt8BE, 0));
    }
    function LCS_get_raw_data(delay: boolean = false): number[] {
        if (delay) {
            // Delay for the integration time to allow reading immediately after the previous read.
            basic.pause((256 - LCS_integration_time_val) * 2.4)
        }

        let div = (256 - LCS_integration_time_val) * 1024
        let rgbc = [0, 0, 0, 0]
        rgbc[0] = I2C_ReadReg16(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.RDATAL)) / div
        rgbc[1] = I2C_ReadReg16(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.GDATAL)) / div
        rgbc[2] = I2C_ReadReg16(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.BDATAL)) / div
        rgbc[3] = I2C_ReadReg16(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.CDATAL)) / div
        if (rgbc[0] > 1) {
            rgbc[0] = 1
        }
        if (rgbc[1] > 1) {
            rgbc[1] = 1
        }
        if (rgbc[2] > 1) {
            rgbc[2] = 1
        }
        if (rgbc[3] > 1) {
            rgbc[3] = 1
        }
        return rgbc
    }

    let font: number[] = [];
    font[0] = 0x0022d422;
    font[1] = 0x0022d422;
    font[2] = 0x0022d422;
    font[3] = 0x0022d422;
    font[4] = 0x0022d422;
    font[5] = 0x0022d422;
    font[6] = 0x0022d422;
    font[7] = 0x0022d422;
    font[8] = 0x0022d422;
    font[9] = 0x0022d422;
    font[10] = 0x0022d422;
    font[11] = 0x0022d422;
    font[12] = 0x0022d422;
    font[13] = 0x0022d422;
    font[14] = 0x0022d422;
    font[15] = 0x0022d422;
    font[16] = 0x0022d422;
    font[17] = 0x0022d422;
    font[18] = 0x0022d422;
    font[19] = 0x0022d422;
    font[20] = 0x0022d422;
    font[21] = 0x0022d422;
    font[22] = 0x0022d422;
    font[23] = 0x0022d422;
    font[24] = 0x0022d422;
    font[25] = 0x0022d422;
    font[26] = 0x0022d422;
    font[27] = 0x0022d422;
    font[28] = 0x0022d422;
    font[29] = 0x0022d422;
    font[30] = 0x0022d422;
    font[31] = 0x0022d422;
    font[32] = 0x00000000;
    font[33] = 0x000002e0;
    font[34] = 0x00018060;
    font[35] = 0x00afabea;
    font[36] = 0x00aed6ea;
    font[37] = 0x01991133;
    font[38] = 0x010556aa;
    font[39] = 0x00000060;
    font[40] = 0x000045c0;
    font[41] = 0x00003a20;
    font[42] = 0x00051140;
    font[43] = 0x00023880;
    font[44] = 0x00002200;
    font[45] = 0x00021080;
    font[46] = 0x00000100;
    font[47] = 0x00111110;
    font[48] = 0x0007462e;
    font[49] = 0x00087e40;
    font[50] = 0x000956b9;
    font[51] = 0x0005d629;
    font[52] = 0x008fa54c;
    font[53] = 0x009ad6b7;
    font[54] = 0x008ada88;
    font[55] = 0x00119531;
    font[56] = 0x00aad6aa;
    font[57] = 0x0022b6a2;
    font[58] = 0x00000140;
    font[59] = 0x00002a00;
    font[60] = 0x0008a880;
    font[61] = 0x00052940;
    font[62] = 0x00022a20;
    font[63] = 0x0022d422;
    font[64] = 0x00e4d62e;
    font[65] = 0x000f14be;
    font[66] = 0x000556bf;
    font[67] = 0x0008c62e;
    font[68] = 0x0007463f;
    font[69] = 0x0008d6bf;
    font[70] = 0x000094bf;
    font[71] = 0x00cac62e;
    font[72] = 0x000f909f;
    font[73] = 0x000047f1;
    font[74] = 0x0017c629;
    font[75] = 0x0008a89f;
    font[76] = 0x0008421f;
    font[77] = 0x01f1105f;
    font[78] = 0x01f4105f;
    font[79] = 0x0007462e;
    font[80] = 0x000114bf;
    font[81] = 0x000b6526;
    font[82] = 0x010514bf;
    font[83] = 0x0004d6b2;
    font[84] = 0x0010fc21;
    font[85] = 0x0007c20f;
    font[86] = 0x00744107;
    font[87] = 0x01f4111f;
    font[88] = 0x000d909b;
    font[89] = 0x00117041;
    font[90] = 0x0008ceb9;
    font[91] = 0x0008c7e0;
    font[92] = 0x01041041;
    font[93] = 0x000fc620;
    font[94] = 0x00010440;
    font[95] = 0x01084210;
    font[96] = 0x00000820;
    font[97] = 0x010f4a4c;
    font[98] = 0x0004529f;
    font[99] = 0x00094a4c;
    font[100] = 0x000fd288;
    font[101] = 0x000956ae;
    font[102] = 0x000097c4;
    font[103] = 0x0007d6a2;
    font[104] = 0x000c109f;
    font[105] = 0x000003a0;
    font[106] = 0x0006c200;
    font[107] = 0x0008289f;
    font[108] = 0x000841e0;
    font[109] = 0x01e1105e;
    font[110] = 0x000e085e;
    font[111] = 0x00064a4c;
    font[112] = 0x0002295e;
    font[113] = 0x000f2944;
    font[114] = 0x0001085c;
    font[115] = 0x00012a90;
    font[116] = 0x010a51e0;
    font[117] = 0x010f420e;
    font[118] = 0x00644106;
    font[119] = 0x01e8221e;
    font[120] = 0x00093192;
    font[121] = 0x00222292;
    font[122] = 0x00095b52;
    font[123] = 0x0008fc80;
    font[124] = 0x000003e0;
    font[125] = 0x000013f1;
    font[126] = 0x00841080;
    font[127] = 0x0022d422;

    let _I2CAddr = 0;
    let _screen = pins.createBuffer(1025);
    let _buf2 = pins.createBuffer(2);
    let _buf3 = pins.createBuffer(3);
    let _buf4 = pins.createBuffer(4);
    let _ZOOM = 1;

    function cmd1(d: number) {
        let n = d % 256;
        pins.i2cWriteNumber(_I2CAddr, n, NumberFormat.UInt16BE);
    }

    function cmd2(d1: number, d2: number) {
        _buf3[0] = 0;
        _buf3[1] = d1;
        _buf3[2] = d2;
        pins.i2cWriteBuffer(_I2CAddr, _buf3);
    }

    function cmd3(d1: number, d2: number, d3: number) {
        _buf4[0] = 0;
        _buf4[1] = d1;
        _buf4[2] = d2;
        _buf4[3] = d3;
        pins.i2cWriteBuffer(_I2CAddr, _buf4);
    }

    function set_pos(col: number = 0, page: number = 0) {
        cmd1(0xb0 | page) // page number
        let c = col * (_ZOOM + 1)
        cmd1(0x00 | (c % 16)) // lower start column address
        cmd1(0x10 | (c >> 4)) // upper start column address    
    }

    // clear bit
    function clrbit(d: number, b: number): number {
        if (d & (1 << b))
            d -= (1 << b)
        return d
    }

    /**
     * show text in OLED
     * @param x is X alis, eg: 0
     * @param y is Y alis, eg: 0
     * @param s is the text will be show, eg: 'Hello!'
     */
    //% blockId="OLED12864_I2C_SHOWSTRING1" block="显示 文本 在 x %x|y %y|文本 %s"
    //% parts=OLED12864_I2C trackArgs=0
    //% group="OLED显示屏" subcategory=执行器
    //% weight=80 blockGap=10 color=#0855AA
    export function showString(x: number, y: number, s: string, color: number = 1) {
        let col2 = 0
        let q = 0
        let ind2 = 0
        for (let r = 0; r < s.length; r++) {
            q = font[s.charCodeAt(r)]
            for (let k = 0; k < 5; k++) {
                col2 = 0
                for (let l = 0; l < 5; l++) {
                    if (q & (1 << (5 * k + l)))
                        col2 |= (1 << (l + 1))
                }
                ind2 = (x + r) * 5 * (_ZOOM + 1) + y * 128 + k * (_ZOOM + 1) + 1
                if (color == 0)
                    col2 = 255 - col2
                _screen[ind2] = col2
                if (_ZOOM)
                    _screen[ind2 + 1] = col2
            }
        }
        set_pos(x * 5, y)
        let ind02 = x * 5 * (_ZOOM + 1) + y * 128
        let buf7 = _screen.slice(ind02, ind2 + 1)
        buf7[0] = 0x40
        pins.i2cWriteBuffer(_I2CAddr, buf7)
    }

    /**
     * show a number in OLED
     * @param x is X alis, eg: 0
     * @param y is Y alis, eg: 0
     * @param num is the number will be show, eg: 12
     * @param color is number color, eg: 1
     */
    //% blockId="OLED12864_I2C_NUMBER" block="显示 数字 在 x %x|y %y|数字 %num"
    //% parts=OLED12864_I2C trackArgs=0
    //% group="OLED显示屏" subcategory=执行器
    //% weight=80 blockGap=10 color=#0855AA
    export function showNumber(x: number, y: number, num: number, color: number = 1) {
        showString(x, y, num.toString(), color)
    }


    export function draw() {
        set_pos()
        pins.i2cWriteBuffer(_I2CAddr, _screen)
    }

    /**
     * clear screen
     */
    //% blockId="OLED12864_I2C_CLEAR" block="清除"
    //% parts=OLED12864_I2C trackArgs=0
    //% group="OLED显示屏" subcategory=执行器
    //% weight=63 blockGap=10 color=#0855AA
    export function clear() {
        _screen.fill(0)
        _screen[0] = 0x40
        draw()
    }

    /**
     * OLED initialize
     * @param addr is i2c addr, eg: 60
     */
    //% blockId="OLED12864_I2C_init" block="初始化 OLED 地址为 %addr"
    //% parts=OLED12864_I2C trackArgs=0
    //% weight=85 blockGap=10
    //% group="OLED显示屏" subcategory=执行器
    //% blockGap=10  color=#0855AA
    export function init(addr: number) {
        _I2CAddr = addr;
        cmd1(0xAE)       // SSD1306_DISPLAYOFF
        cmd1(0xA4)       // SSD1306_DISPLAYALLON_RESUME
        cmd2(0xD5, 0xF0) // SSD1306_SETDISPLAYCLOCKDIV
        cmd2(0xA8, 0x3F) // SSD1306_SETMULTIPLEX
        cmd2(0xD3, 0x00) // SSD1306_SETDISPLAYOFFSET
        cmd1(0 | 0x0)    // line #SSD1306_SETSTARTLINE
        cmd2(0x8D, 0x14) // SSD1306_CHARGEPUMP
        cmd2(0x20, 0x00) // SSD1306_MEMORYMODE
        cmd3(0x21, 0, 127) // SSD1306_COLUMNADDR
        cmd3(0x22, 0, 63)  // SSD1306_PAGEADDR
        cmd1(0xa0 | 0x1) // SSD1306_SEGREMAP
        cmd1(0xc8)       // SSD1306_COMSCANDEC
        cmd2(0xDA, 0x12) // SSD1306_SETCOMPINS
        cmd2(0x81, 0xCF) // SSD1306_SETCONTRAST
        cmd2(0xd9, 0xF1) // SSD1306_SETPRECHARGE
        cmd2(0xDB, 0x40) // SSD1306_SETVCOMDETECT
        cmd1(0xA6)       // SSD1306_NORMALDISPLAY
        cmd2(0xD6, 1)    // zoom on
        cmd1(0xAF)       // SSD1306_DISPLAYON
        clear()
        _ZOOM = 1
    }

    /**
     * A NeoPixel strip
     */
    export class Strip {
        buf: Buffer;
        pin: DigitalPin;
        // TODO: encode as bytes instead of 32bit
        brightness: number;
        start: number; // start offset in LED strip
        _length: number; // number of LEDs
        _mode: NeoPixelMode;
        _matrixWidth: number; // number of leds in a matrix - if any

        /**
         * Shows all LEDs to a given color (range 0-255 for r, g, b).
         * @param rgb RGB color of the LED
         */
        //% blockId="neopixel_set_strip_color" block="%strip|显示 颜色 %rgb=neopixel_colors"
        //% strip.defl=strip
        //% parts="neopixel" subcategory=执行器 group="彩灯"
        //% color=#2699BF blockGap=10
        showColor(rgb: number) {
            rgb = rgb >> 0;
            this.setAllRGB(rgb);
            this.show();
        }

        /**
         * Shows a rainbow pattern on all LEDs.
         * @param startHue the start hue value for the rainbow, eg: 1
         * @param endHue the end hue value for the rainbow, eg: 360
         */
        //% blockId="neopixel_set_strip_rainbow" block="%strip|显示 彩虹 从 %startHue|到 %endHue"
        //% strip.defl=strip
        //% parts="neopixel" subcategory=执行器 group="彩灯" 
        //% color=#2699BF blockGap=10
        showRainbow(startHue: number = 1, endHue: number = 360) {
            if (this._length <= 0) return;

            startHue = startHue >> 0;
            endHue = endHue >> 0;
            const saturation = 100;
            const luminance = 50;
            const steps = this._length;
            const direction = HueInterpolationDirection.Clockwise;

            //hue
            const h1 = startHue;
            const h2 = endHue;
            const hDistCW = ((h2 + 360) - h1) % 360;
            const hStepCW = Math.idiv((hDistCW * 100), steps);
            const hDistCCW = ((h1 + 360) - h2) % 360;
            const hStepCCW = Math.idiv(-(hDistCCW * 100), steps);
            let hStep: number;
            if (direction === HueInterpolationDirection.Clockwise) {
                hStep = hStepCW;
            } else if (direction === HueInterpolationDirection.CounterClockwise) {
                hStep = hStepCCW;
            } else {
                hStep = hDistCW < hDistCCW ? hStepCW : hStepCCW;
            }
            const h1_100 = h1 * 100; //we multiply by 100 so we keep more accurate results while doing interpolation

            //sat
            const s1 = saturation;
            const s2 = saturation;
            const sDist = s2 - s1;
            const sStep = Math.idiv(sDist, steps);
            const s1_100 = s1 * 100;

            //lum
            const l1 = luminance;
            const l2 = luminance;
            const lDist = l2 - l1;
            const lStep = Math.idiv(lDist, steps);
            const l1_100 = l1 * 100

            //interpolate
            if (steps === 1) {
                this.setPixelColor(0, hsl(h1 + hStep, s1 + sStep, l1 + lStep))
            } else {
                this.setPixelColor(0, hsl(startHue, saturation, luminance));
                for (let t = 1; t < steps - 1; t++) {
                    const u = Math.idiv((h1_100 + t * hStep), 100) + 360;
                    const v = Math.idiv((s1_100 + t * sStep), 100);
                    const w = Math.idiv((l1_100 + t * lStep), 100);
                    this.setPixelColor(t, hsl(u, v, w));
                }
                this.setPixelColor(steps - 1, hsl(endHue, saturation, luminance));
            }
            this.show();
        }


        /**
         * Set LED to a given color (range 0-255 for r, g, b).
         * You need to call ``show`` to make the changes visible.
         * @param pixeloffset position of the NeoPixel in the strip,eg: 1
         * @param rgb RGB color of the LED
         */
        //% blockId="neopixel_set_pixel_color" block="%strip|设置 像素 颜色 在 %pixeloffset|到 %rgb=neopixel_colors"
        //% strip.defl=strip
        //% parts="neopixel" subcategory=执行器 group="彩灯"
        //% color=#2699BF blockGap=10
        setPixelColor(pixeloffset: number, rgb: number): void {
            this.setPixelRGB(pixeloffset >> 0, rgb >> 0);
        }

        /**
         * Send all the changes to the strip.
         */
        //% blockId="neopixel_show" block="%strip|显示"
        //% strip.defl=strip
        //% parts="neopixel" subcategory=执行器 group="彩灯"
        //% color=#2699BF blockGap=10
        show() {
            // only supported in beta
            // ws2812b.setBufferMode(this.pin, this._mode);
            ws2812b.sendBuffer(this.buf, this.pin);
        }

        /**
         * Turn off all LEDs.
         * You need to call ``show`` to make the changes visible.
         */
        //% blockId="neopixel_clear" block="%strip|清除"
        //% strip.defl=strip
        //% parts="neopixel" subcategory=执行器 group="彩灯"
        //% color=#2699BF blockGap=10
        clear(): void {
            const stride = this._mode === NeoPixelMode.RGBW ? 4 : 3;
            this.buf.fill(0, this.start * stride, this._length * stride);
        }

        /**
         * Set the brightness of the strip. This flag only applies to future operation.
         * @param brightness a measure of LED brightness in 0-255. eg: 255
         */
        //% blockId="neopixel_set_brightness" block="%strip|设置 亮度 %brightness"
        //% strip.defl=strip
        //% parts="neopixel" subcategory=执行器 group="彩灯"
        //% color=#2699BF blockGap=10
        setBrightness(brightness: number): void {
            this.brightness = brightness & 0xff;
        }

        /**
         * Set the pin where the neopixel is connected, defaults to P0.
         */
        //% parts="neopixel" subcategory=执行器 group="彩灯"
        //% color=#2699BF blockGap=10
        setPin(pin: DigitalPin): void {
            this.pin = pin;
            pins.digitalWritePin(this.pin, 0);
            // don't yield to avoid races on initialization
        }



        private setBufferRGB(offset: number, red: number, green: number, blue: number): void {
            if (this._mode === NeoPixelMode.RGB_RGB) {
                this.buf[offset + 0] = red;
                this.buf[offset + 1] = green;
            } else {
                this.buf[offset + 0] = green;
                this.buf[offset + 1] = red;
            }
            this.buf[offset + 2] = blue;
        }

        private setAllRGB(rgb: number) {
            let red = unpackR(rgb);
            let green = unpackG(rgb);
            let blue = unpackB(rgb);

            const br3 = this.brightness;
            if (br3 < 255) {
                red = (red * br3) >> 8;
                green = (green * br3) >> 8;
                blue = (blue * br3) >> 8;
            }
            const end3 = this.start + this._length;
            const stride6 = this._mode === NeoPixelMode.RGBW ? 4 : 3;
            for (let i5 = this.start; i5 < end3; ++i5) {
                this.setBufferRGB(i5 * stride6, red, green, blue)
            }
        }
        private setAllW(white: number) {
            if (this._mode !== NeoPixelMode.RGBW)
                return;

            let br4 = this.brightness;
            if (br4 < 255) {
                white = (white * br4) >> 8;
            }
            let buf8 = this.buf;
            let end4 = this.start + this._length;
            for (let i6 = this.start; i6 < end4; ++i6) {
                let ledoffset3 = i6 * 4;
                buf8[ledoffset3 + 3] = white;
            }
        }
        private setPixelRGB(pixeloffset: number, rgb: number): void {
            if (pixeloffset < 0
                || pixeloffset >= this._length)
                return;

            let stride7 = this._mode === NeoPixelMode.RGBW ? 4 : 3;
            pixeloffset = (pixeloffset + this.start) * stride7;

            let red2 = unpackR(rgb);
            let green2 = unpackG(rgb);
            let blue2 = unpackB(rgb);

            let br5 = this.brightness;
            if (br5 < 255) {
                red2 = (red2 * br5) >> 8;
                green2 = (green2 * br5) >> 8;
                blue2 = (blue2 * br5) >> 8;
            }
            this.setBufferRGB(pixeloffset, red2, green2, blue2)
        }
        private setPixelW(pixeloffset: number, white: number): void {
            if (this._mode !== NeoPixelMode.RGBW)
                return;

            if (pixeloffset < 0
                || pixeloffset >= this._length)
                return;

            pixeloffset = (pixeloffset + this.start) * 4;

            let br6 = this.brightness;
            if (br6 < 255) {
                white = (white * br6) >> 8;
            }
            let buf9 = this.buf;
            buf9[pixeloffset + 3] = white;
        }
    }

    /**
     * Create a new NeoPixel driver for `numleds` LEDs.
     * @param pin the pin where the neopixel is connected.
     * @param numleds number of leds in the strip, eg: 8,30,60,64
     */
    //% blockId="neopixel_create" block="灯环 在 端口 %pin|用 %numleds| leds"
    //% parts="neopixel" subcategory=执行器 group="彩灯"
    //% trackArgs=0,2
    //% blockSetVariable=strip
    //% color=#2699BF blockGap=10
    //% weight=51
    export function create(pin: DigitalPin, numleds: number): Strip {
        let strip2 = new Strip();
        let stride8 = NeoPixelMode.RGBW ? 4 : 3;
        strip2.buf = pins.createBuffer(numleds * stride8);
        strip2.start = 0;
        strip2._length = numleds;
        strip2._mode = NeoPixelMode.RGB;
        strip2._matrixWidth = 0;
        strip2.setBrightness(128)
        strip2.setPin(pin)
        return strip2;
    }

    /**
     * Converts red, green, blue channels into a RGB color
     * @param red value of the red channel between 0 and 255. eg: 255
     * @param green value of the green channel between 0 and 255. eg: 255
     * @param blue value of the blue channel between 0 and 255. eg: 255
     */
    //% blockId="neopixel_rgb" block="红 %red|绿 %green|蓝 %blue"
    //% parts="neopixel" subcategory=执行器 group="彩灯"
    //% weight=32
    //% color=#2699BF blockGap=10
    export function rgb(red: number, green: number, blue: number): number {
        return packRGB(red, green, blue);
    }

    /**
     * Gets the RGB value of a known color
    */
    //% blockId="neopixel_colors" block="%color"
    //% parts="neopixel" subcategory=执行器 group="彩灯"
    //% weight=32
    //% color=#2699BF blockGap=10
    export function colors(color: NeoPixelColors): number {
        return color;
    }

    function packRGB(a: number, b: number, c: number): number {
        return ((a & 0xFF) << 16) | ((b & 0xFF) << 8) | (c & 0xFF);
    }

    function unpackR(rgb: number): number {
        let r3 = (rgb >> 16) & 0xFF;
        return r3;
    }

    function unpackG(rgb: number): number {
        let g3 = (rgb >> 8) & 0xFF;
        return g3;
    }

    function unpackB(rgb: number): number {
        let b3 = (rgb) & 0xFF;
        return b3;
    }

    export function hsl(h: number, s: number, l: number): number {
        h = Math.round(h);
        s = Math.round(s);
        l = Math.round(l);

        h = h % 360;
        s = Math.clamp(0, 99, s);
        l = Math.clamp(0, 99, l);
        let c2 = Math.idiv((((100 - Math.abs(2 * l - 100)) * s) << 8), 10000); //chroma, [0,255]
        let h12 = Math.idiv(h, 60);//[0,6]
        let h22 = Math.idiv((h - h12 * 60) * 256, 60);//[0,255]
        let temp = Math.abs((((h12 % 2) << 8) + h22) - 256);
        let x = (c2 * (256 - (temp))) >> 8;//[0,255], second largest component of this color
        let r$: number;
        let g$: number;
        let b$: number;
        if (h12 == 0) {
            r$ = c2; g$ = x; b$ = 0;
        } else if (h12 == 1) {
            r$ = x; g$ = c2; b$ = 0;
        } else if (h12 == 2) {
            r$ = 0; g$ = c2; b$ = x;
        } else if (h12 == 3) {
            r$ = 0; g$ = x; b$ = c2;
        } else if (h12 == 4) {
            r$ = x; g$ = 0; b$ = c2;
        } else if (h12 == 5) {
            r$ = c2; g$ = 0; b$ = x;
        }
        let m2 = Math.idiv((Math.idiv((l * 2 << 8), 100) - c2), 2);
        let r4 = r$ + m2;
        let g4 = g$ + m2;
        let b4 = b$ + m2;
        return packRGB(r4, g4, b4);
    }

    export enum HueInterpolationDirection {
        Clockwise,
        CounterClockwise,
        Shortest
    }
}
