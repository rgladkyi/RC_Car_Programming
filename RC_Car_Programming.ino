// Under GNU GPL v2
// Copyright (c) Roman Gladkyi

#include <Servo.h> 

#define BOARD_TYPE_ARDUINO_NANO_V3_ATMEGA_328 1001
#define BOARD_TYPE_ARDUINO_UNO_V3_ATMEGA_328 1002

#define BOARD_TYPE BOARD_TYPE_ARDUINO_NANO_V3_ATMEGA_328

#if BOARD_TYPE == BOARD_TYPE_ARDUINO_NANO_V3_ATMEGA_328
#define PIN_NAME_TX1 1
#define PIN_NAME_RX0 0
#define PIN_NAME_D2 2
#define PIN_NAME_D3 3
#define PIN_NAME_D4 4
#define PIN_NAME_D5 5
#define PIN_NAME_D6 6
#define PIN_NAME_D7 7
#define PIN_NAME_D8 8
#define PIN_NAME_D9 9
#define PIN_NAME_D10 10
#define PIN_NAME_D11 12
#define PIN_NAME_D12 13
#define PIN_NAME_D14 14
#define PIN_NAME_D15 15
#define PIN_NAME_D16 16
#define PIN_NAME_D17 17
#define PIN_NAME_D18 18
#else if BOARD_TYPE == BOARD_TYPE_ARDUINO_UNO_V3_ATMEGA_328
#error Not supported board type.
#endif

#define RC_CONFIG_HIMOTO_CENTRO_1_18 2001
#define RC_CONFIG_HSP_MINI_1_18 2002

#define RC_CONFIG RC_CONFIG_HIMOTO_CENTRO_1_18

#if RC_CONFIG == RC_CONFIG_HIMOTO_CENTRO_1_18
#define RC_CHANNEL_DELAY_UPDATE_MS 50
#define RC_CAR_ACTIONS_GENERIC_DELAY_MS 500 

#define PIN_RC_CAR_ON_OFF_SWITCH 7
#define PIN_RC_CAR_BIND_RECEIVER 8

#define PIN_RC_CAR_RCVR_SERVO 14
#define PIN_RC_CAR_RCVR_MOTOR 15

#define PIN_RC_CAR_DRIVE_SERVO 16
#define PIN_RC_CAR_DRIVE_MOTOR 17

#else if RC_CONFIG == RC_CONFIG_HSP_MINI_1_18
#define RC_CHANNEL_DELAY_UPDATE_MS 50
#define RC_CAR_ACTIONS_GENERIC_DELAY_MS 500 

#define PIN_RC_CAR_ON_OFF_SWITCH 3
#define PIN_RC_CAR_BIND_RECEIVER 8

#define PIN_RC_CAR_RCVR_SERVO 17
#define PIN_RC_CAR_RCVR_MOTOR 16

#define PIN_RC_CAR_DRIVE_SERVO 15
#define PIN_RC_CAR_DRIVE_MOTOR 14

#endif

#define ENABLE_INPUT_PIN(pin) pinMode((pin), INPUT)
#define ENABLE_OUTPUT_PIN(pin) pinMode((pin), OUTPUT)
#define ENABLE_INPUT_PINS(pins, count) for (int iter = 0; iter < count; iter++) ENABLE_INPUT_PIN(pins[iter])
#define ENABLE_OUTPUT_PINS(pins, count) for (int iter = 0; iter < count; iter++) ENABLE_OUTPUT_PIN(pins[iter])

#define PIN_READ(pin) digitalRead((pin))
#define PIN_WRITE(pin, value) digitalWrite((pin), (value))

#define PIN_IS_STATE_LOW(pin) (PIN_READ((pin)) == LOW)
#define PIN_IS_STATE_HIGH(pin) (PIN_READ((pin)) == HIGH)

#define PIN_WRITE_LOW(pin) PIN_WRITE((pin), LOW)
#define PIN_WRITE_HIGH(pin) PIN_WRITE((pin), HIGH)

#define PIN_PULSE_IN_LOW(pin) pulseIn((pin), LOW)
#define PIN_PULSE_IN_HIGH(pin) pulseIn((pin), HIGH)

#define PIN_PULSE_OUT_LOW(pin, value) \
    PIN_WRITE_LOW((pin)); \
    delayMicroseconds(value); \
    PIN_WRITE_HIGH((pin));

#define PIN_PULSE_OUT_HIGH(pin, value) \
    PIN_WRITE_HIGH((pin)); \
    delayMicroseconds(value); \
    PIN_WRITE_LOW((pin));

#define rc_channel_read_value(pin) PIN_PULSE_IN_HIGH((pin))
#define rc_channel_write_value(pin, value) PIN_PULSE_OUT_HIGH((pin), (value))

#define PRINT(message) Serial.println((#message " = " message))

#define rc_channel_redirect(pin_from, pin_to) \
	rc_channel_write_value((pin_to), rc_channel_read_value((pin_from)));
	
#define rc_channels_wait_update() delay(RC_CHANNEL_DELAY_UPDATE_MS);


#define PIN_RC_RCVR_PULSEIN_TIMEOUT_USEC 1 * 1000 * 1000

typedef unsigned char byte;
typedef unsigned short ushort;
typedef unsigned int uint;
typedef unsigned long ulong;

Servo rc_drive_servo;
Servo rc_drive_motor;

static void init_board()
{
    // initialize serial communication at 9600 bits per second:
    Serial.begin(9600);
}

static void uninit_board()
{
}

void setup()
{
	init_board();
	
    pinMode(PIN_RC_CAR_ON_OFF_SWITCH, OUTPUT);
    pinMode(PIN_RC_CAR_BIND_RECEIVER, OUTPUT);
    pinMode(PIN_RC_CAR_RCVR_SERVO, INPUT);
    pinMode(PIN_RC_CAR_RCVR_MOTOR, INPUT);
    
	rc_drive_servo.attach(PIN_RC_CAR_DRIVE_SERVO);
	rc_drive_motor.attach(PIN_RC_CAR_DRIVE_MOTOR);
	
	pinMode(PIN_RC_CAR_DRIVE_SERVO, OUTPUT);
    pinMode(PIN_RC_CAR_DRIVE_MOTOR, OUTPUT);
}

int rc_start()
{
#if RC_CONFIG == RC_CONFIG_HIMOTO_CENTRO_1_18
	digitalWrite(PIN_RC_CAR_ON_OFF_SWITCH, HIGH);
#else if RC_CONFIG == RC_CONFIG_HSP_MINI_1_18
	digitalWrite(PIN_RC_CAR_ON_OFF_SWITCH, HIGH);
	delay(20);
	digitalWrite(PIN_RC_CAR_ON_OFF_SWITCH, LOW);
	delay(20);
	digitalWrite(PIN_RC_CAR_ON_OFF_SWITCH, HIGH);
#endif
    delay(RC_CAR_ACTIONS_GENERIC_DELAY_MS);
}

int rc_bind_receiver()
{
    digitalWrite(PIN_RC_CAR_ON_OFF_SWITCH, LOW);
    delay(500);
    digitalWrite(PIN_RC_CAR_BIND_RECEIVER, HIGH);
    delay(500);
    digitalWrite(PIN_RC_CAR_ON_OFF_SWITCH, HIGH);
    delay(500);
    
    delay(3000);
    
    digitalWrite(PIN_RC_CAR_ON_OFF_SWITCH, LOW);
    delay(500);
    digitalWrite(PIN_RC_CAR_BIND_RECEIVER, LOW);
    delay(500);
    digitalWrite(PIN_RC_CAR_ON_OFF_SWITCH, HIGH);
    delay(500);
}

int rc_rcvr_read()
{
	static unsigned long servo_state_prev = 1500;
	unsigned long servo_state = pulseIn(PIN_RC_CAR_RCVR_SERVO, HIGH);
	
	if (servo_state > 1900 || servo_state < 1100)
	{
		servo_state = 1500;		
	}
	
	if (abs(servo_state - servo_state_prev) > 10)
	{
		rc_drive_servo.writeMicroseconds(servo_state);	
	}
	
	servo_state_prev = servo_state;

	static unsigned long motor_state_prev = 1500;
	unsigned long motor_state = pulseIn(PIN_RC_CAR_RCVR_MOTOR, HIGH);
	
	if (motor_state > 1800 || motor_state < 1150)
	{
		motor_state = 1500;		
	}
	
	if (abs(motor_state - motor_state_prev) > 10)
	{
		rc_drive_motor.writeMicroseconds(motor_state);
	}
	
	servo_state_prev = servo_state;
	
	delay(1);

	//delay(10);
	//Serial.print("Servo state: ");
	//Serial.println(servo_state);
	
	
	//Serial.print("Motor state: ");
	//Serial.println(motor_state);	
}

#define NO_BRAND_25A_ESC_COEF_FORW_SPEED_MIN 1546
#define NO_BRAND_25A_ESC_COEF_FORW_SPEED_MAX 1900

#define NO_BRAND_25A_ESC_COEF_BACKW_SPEED_MIN 1450
#define NO_BRAND_25A_ESC_COEF_BACKW_SPEED_MAX 11s00

void loop()
{
	PRINT("start");
    rc_start();
	
    while (1)
    {
		//rc_channel_write_value(PIN_RC_CAR_DRIVE_SERVO, 1580);
		//rc_channel_write_value(PIN_RC_CAR_DRIVE_MOTOR, 1546);
		//delay(50);
		
		rc_rcvr_read();
		
		//delay(50);
		//delay(500);
		//Serial.print("servo");
    	//int servo_state = (rc_channel_read_value(PIN_RC_CAR_RCVR_SERVO));
		//rc_channel_write_value(PIN_RC_CAR_DRIVE_SERVO, servo_state);
		//Serial.println(servo_state);
		
		//Serial.print("esc");
		//int esc_state = (rc_channel_read_value(PIN_RC_CAR_RCVR_MOTOR));
		//rc_channel_write_value(PIN_RC_CAR_DRIVE_MOTOR, esc_state);
		//Serial.println(esc_state);
		//delay(50);
		
		//rc_channel_redirect(PIN_RC_CAR_RCVR_SERVO, PIN_RC_CAR_DRIVE_SERVO);
		//rc_channel_redirect(PIN_RC_CAR_RCVR_MOTOR, PIN_RC_CAR_DRIVE_MOTOR);
		
		//rc_channels_wait_update();
	}
}

