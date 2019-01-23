#include <arduino.h>
#include <PS2X_lib.h>  //for v1.6
#include <U8x8lib.h>
#include <MemoryFree.h>
#include <SoftwareSerial.h>
#include <FastCRC.h>



/* define debug stuff */
#define DEBUG_FREE_MEMORY 0


// set 5 if 16Mhz Arduino, set 3.3 if 8Mhz Arduino
#define IO_VOLTAGE_LEVEL 3.3
#define VOLTAGE_PIN_RAW A0
// #define VOLTAGE_DIVIDE_RESISTOR_1 21600
#define VOLTAGE_DIVIDE_RESISTOR_1 22000
#define VOLTAGE_DIVIDE_RESISTOR_2 9600
#define VOLTAGE_READ_INTERVAL_MS 1000
unsigned long lastVoltagePrinted=0;

// we can read only on top of 3.76V. Otherwise our reference 3.3V will drop and invalidate the results


//#ifdef U8X8_HAVE_HW_SPI
//#include <SPI.h>
//#endif

/*PS2 PINs*/
#define PS2_DAT        2   //14
#define PS2_CMD        4  //15
#define PS2_SEL        3  //16
#define PS2_CLK 	   5 //17


/* BLE */
#define BT_RX 10
#define BT_TX 11
#define BLE_BAUDRATE 19200
#define BLE_DISCOVER_ENTRY_COUNT 2
#define BLE_DISCOVER_NAME_LENGTH 14
#define BLE_DISCOVER_ADDRESS_LENGTH 12

/* SERIAL */
#define SERIAL_BAUD 19200



/* Free memory output */
#define DEBUG_PRINT_FREE_MEM_TIMER 2000

/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/

#define pressures   true
#define rumble      false // rumble motors are removed, so set it to false

PS2X ps2x; // create PS2 Controller Class

//right now, the library does NOT support hot pluggable controllers, meaning
//you must always either restart your Arduino after you connect the controller,
//or call config_gamepad(pins) again after connecting the controller.

int error = 0;
byte type = 0;
byte vibrate = 0;

long lastMillis = 0;
unsigned long lastMemoryStatsMillis = 0;

//U8X8_SSD1306_128X32_UNIVISION_SW_I2C display(/* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // Adafruit Feather ESP8266/32u4 Boards + FeatherWing OLED
U8X8_SH1106_128X64_NONAME_HW_I2C display (/* reset=*/ U8X8_PIN_NONE);

bool isModelChoosen=true;


// create serial communication to BLE HM-10 / HM-11 device
SoftwareSerial bleSerial (BT_RX, BT_TX); // RX, TX


// Protocol definition
#define PROT_ARRAY_LENGTH 17
#define PROT_STICK_LX 0
#define PROT_STICK_LY 1
#define PROT_STICK_RX 2
#define PROT_STICK_RY 3
#define PROT_BTN_SQUARE 4
#define PROT_BTN_CROSS 5
#define PROT_BTN_TRIANGLE 6
#define PROT_BTN_CIRCLE 7
#define PROT_BTN_L1 8
#define PROT_BTN_L2 9
#define PROT_BTN_R1 10
#define PROT_BTN_R2 11
#define PROT_UP 12
#define PROT_DONW 13
#define PROT_LEFT 14
#define PROT_RIGHT 15
#define PROT_DIGITAL_BUTTONS 16
#define PROT_DIGITAL_START 0
#define PROT_DIGITAL_SELECT 1
#define PROT_DIGITAL_STICK_BTN_L 2
#define PROT_DIGITAL_STICK_BTN_R 3
//#define PROT_CRC 16
//#define PROT_CR 17
//#define PROT_LF 18

// BT send interval
#define BT_MIN_SEND_INTERVAL_MILLIS 50
unsigned long btLastSend=0;

// CRC
FastCRC8 CRC8;

// stick adjustment
#define DEAD_ZONE 20

#define TRIM_LX 0
#define TRIM_LY 1
#define TRIM_RX 2
#define TRIM_RY 3
char sticksTrim[] = {0,0,0,0};

bool stickReverse[] = {false,false,false,false};


void printDebug()
{
	//Serial.print("Stick Values:");
	Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
	Serial.print(",");
	Serial.print(ps2x.Analog(PSS_LX), DEC);
	Serial.print(",");
	Serial.print(ps2x.Analog(PSS_RY), DEC);
	Serial.print(",");
	Serial.println(ps2x.Analog(PSS_RX), DEC);

	Serial.print(F(" voltage: "));
	Serial.println(readVoltage(VOLTAGE_PIN_RAW, (float) VOLTAGE_DIVIDE_RESISTOR_1, (float) VOLTAGE_DIVIDE_RESISTOR_2));


}

void displayDebug()
{
	if (millis() - lastVoltagePrinted > (unsigned long) VOLTAGE_READ_INTERVAL_MS)
	{
		display.setCursor(5, 5);
		display.print(readVoltage(VOLTAGE_PIN_RAW, (float) VOLTAGE_DIVIDE_RESISTOR_1, (float) VOLTAGE_DIVIDE_RESISTOR_2));
		lastVoltagePrinted = millis();
	}

}

void displayWriteEntry(uint8_t lineNumber, char* stringToWrite)
{
	display.clearLine(2+lineNumber);
	display.setCursor(3, 2+lineNumber);
	display.print(stringToWrite);


}

void setup() {

	Serial.begin(SERIAL_BAUD);
	Serial.println(F("starting"));

	stickReverse[TRIM_LY]=true;
	display.begin();
	delay (200);
	display.setPowerSave(0);
	//display.clear();
	display.setFont(u8x8_font_chroma48medium8_r);
////	display.setContrast(255);

	display.drawString(2, 0, "starting");
	display.drawString(1, 3, "searching");
	display.drawString(0, 4, "controller!");

	// delay(2000);
	//delay(300); //added delay to give wireless ps2 module some time to startup, before configuring it

	//CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************

//  setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
	error = -1;
	while (error != 0)
	{
		error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);


		if (error == 0)
		{
			Serial.print(F("Found Controller, configured successful "));
			Serial.print(F("pressures = "));
			if (pressures)
				Serial.println(F("true "));
			else
				Serial.println(F("false"));
			Serial.print("rumble = ");
			if (rumble)
				Serial.println("true)");
			else {
				Serial.println("false");
				Serial.println(F("Try out all the buttons, X will vibrate the controller, faster as you press harder;"));
				Serial.println(
						"holding L1 or R1 will print out the analog stick values.");
				Serial.println(F("Note: Go to www.billporter.info for updates and to report bugs."));
			}
		}
		else if (error == 1)
		{
			Serial.println(F("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips"));
		}
		else if (error == 2)
		{
			Serial.println(F("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips"));
		}

		else if (error == 3)
		{
			Serial.println(F("Controller refusing to enter Pressures mode, may not support it. "));
		}




//		type = ps2x.readType();
//		switch (type) {
//		case 0:
//			Serial.print("Unknown Controller type found ");
//			break;
//		case 1:
//			Serial.print("DualShock Controller found ");
//			break;
//		case 2:
//			Serial.print("GuitarHero Controller found ");
//			break;
//		case 3:
//			Serial.print("Wireless Sony DualShock Controller found ");
//			break;
//		}
		delay(300);
	}
	pinMode(13, OUTPUT);

	display.clearDisplay();
	display.drawString(1, 0, "controller");
	display.drawString(3, 1, "found");

	Serial.print(ps2x.Analog(PSS_LX), DEC);
	Serial.print(" ");
	Serial.print(ps2x.Analog(PSS_LY), DEC);
	Serial.print(" ");
	Serial.print(ps2x.Analog(PSS_RX), DEC);
	Serial.print(" ");
	Serial.print(ps2x.Analog(PSS_RY), DEC);
	Serial.print("\n");


	//delay(2000);
	bleSerial.begin(BLE_BAUDRATE);
	delay(200);
	bleSendATWakeUp();
	delay(200);
	bleReadSerial();
	Serial.println(F("AT+ROLE1"));
	bleSerial.println(F("AT+ROLE1"));
	delay(200);
	bleReadSerial();

//	bleSerial.println("AT+BAUD1");
//	delay(200);
//	bleReadSerial();

	Serial.println(F("AT+POWE2"));
	bleSerial.println(F("AT+POWE2"));
	delay(100);
	bleReadSerial();

	Serial.println(F("AT+RESET"));
	bleSerial.println(F("AT+RESET"));
	delay(100);
	bleReadSerial();

	Serial.println(F("AT"));
	bleSerial.println(F("AT"));
	delay(100);
	bleReadSerial();

	bleReadSerial();

	Serial.println(F("AT+NAMESENDER"));
	bleSerial.println(F("AT+NAMESENDER"));
	delay(100);
	bleReadSerial();


//	bleSerial.println("AT+CONA20C38FEF6C33");
//	delay(2000);
	// bleSerial.println("AT+CONN1");

	bleScan();
	delay(5);
	//bleReadSerial();
	Serial.println(F("setup end"));
}




void bleScan ()
{

	printSerialFreeMemory();
	displayWriteEntry(2, "Hallo");
	displayWriteEntry(4, "test");
	//char discoveryNames[BLE_DISCOVER_ENTRY_COUNT][BLE_DISCOVER_NAME_LENGTH];
	//char discoveryAddress[BLE_DISCOVER_ENTRY_COUNT][BLE_DISCOVER_ADDRESS_LENGTH];

	printSerialFreeMemory();



	//set results all empty
//	for (int entry=0; entry < BLE_DISCOVER_ENTRY_COUNT; entry ++)
//	{
//		//memset(discoveryNames, 0, sizeof(discoveryAddress));
//		memset(discoveryAddress, 0, sizeof(discoveryAddress));
//	}

	printSerialFreeMemory();
	// send discovery command
	bleSerial.println(F("AT+DISC?"));

	// we save our response line here
	String responseLine = "";
	responseLine.reserve(64);


	printSerialFreeMemory();
//	char tmpResponse[64]= "";


	// clean tmpResponse
//	memset(tmpResponse, 0, sizeof(tmpResponse));

	bool isResponseCompletelyRead = false;
	int index = 0;
	int lineNumber=0;
	// read as long as response is not completely read
	while (!isResponseCompletelyRead)
	{
		printSerialFreeMemory();
		if (bleSerial.available() >0)
		{
			// we have data in serial waiting
			responseLine = bleSerial.readStringUntil('\r');
			Serial.print(lineNumber);

			Serial.println(responseLine);
//			// search for Name starting with MYRC
//			if (strstr(tmpResponse, "OK+NAME:MYRC_"))
//			{
//				//strcat(discoveryNames[index], tmpResponse[13], sizeof(discoveryNames[index]));
////				Serial.println(sizeof(tmpResponse));
//			}
//
//
//
			// check for end sequence
//			if (strstr(responseLine, "OK+DISCE"))
//			{
//				isResponseCompletelyRead=true;
//			}
			if (responseLine.indexOf(F("OK+DISCE")!=-1))
			{
				isResponseCompletelyRead=true;
			}

//			int firstDelimiter = line.indexOf(':');
//			if (firstDelimiter != -1 )
//			{
//				// we jave a match. Line looks like OK+DIS0:78C5E56D8BCDOK+RSSI:-045
//				// get the address
//				Serial.println( firstDelimiter);
//				int secondDelimter = line.lastIndexOf(':');
//				String address = line.substring(firstDelimiter+1, line.indexOf('+',firstDelimiter));
//				//int rssi = line.substring(line.lastIndexOf(':')+1);
//
//				Serial.print("address: ");
//				Serial.println(address);
//				Serial.print("rssi: ");
//				Serial.println(rssi);


//			}



			lineNumber++;


		}



	}

	//if (bleSerial.available() >0)
//	{
//		response = response + bleSerial.readString();
//		if (! response.indexOf("OK+DISCE") > 0)
//		{
//
//		}
//	}

	printSerialFreeMemory();

}

/* send long text for wakeup via AT*/
void bleSendATWakeUp ()
{
	bleSerial.println(F("sksnlskndvlknslkvnslvknklnlnlkndabcvsidbvsksnlskndvlknslkvnslvknklnlnlkndabcvsidbvsksnlskndvlknslkvnslvknklnlnlkndabcvsidbvsksnlskndvlknslkvnslvknklnlnlkndabcvsidbvAT"));
	bleSerial.println(F("AT"));
}

void bleReadSerial()
{
	if (bleSerial.available() > 0)
	{
		Serial.println(bleSerial.readString());
	}

}


void printSerialFreeMemory() {
	if (millis() - (unsigned long) DEBUG_PRINT_FREE_MEM_TIMER > lastMemoryStatsMillis)
	{
		Serial.print(millis());
		Serial.print(F(" - freeMemory()="));
		Serial.println(freeMemory());
		lastMemoryStatsMillis = millis();
	}

}

/**
 * reads voltage on pin. It's build for voltage divider circuit with 2 resistors.
 */
float readVoltage(int myPin, float resistor1, float resistor2)
{
	float voltageOnPin = (float) map(analogRead(myPin), 0, 1023, 0, (float) IO_VOLTAGE_LEVEL*1000)/1000;
	float ratio = (float) (resistor2 / (resistor1 + resistor2));
//	Serial.print("onPin: ");
//	Serial.print(analogRead(myPin));
//	Serial.print("PinVoltage: ");
//	Serial.print(voltageOnPin);
//	Serial.print("ratio: ");
//	Serial.println(ratio);
	//delay (100);

	return (float) voltageOnPin / ratio;
}

void displayError()
{
	Serial.println(F("in displayError"));
	switch (error)
	{
		case 1:
			display.clear();
			display.print(F("no controller!"));
			Serial.println(F("no controller!"));

			break;
		case 2:
			display.clear();
			display.print(F("no cmd"));
			Serial.println(F("no cmd"));
			break;
		}


}
void blinkErrorCode() {
	switch (error)
	{
	case 1:
		digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
		delay(1000);              // wait for a second
		digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
		delay(1000);              // wait for a second
		break;
	case 2:
		digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
		delay(500);              // wait for a second
		digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
		delay(500);              // wait for a second
		break;
	}

}

//void counterDisplay()
//{
//	long currentMillis = millis();
//	if (currentMillis > lastMillis + 500)
//	{
//		display.clearDisplay();
//		// text display tests
//		display.setTextSize(1);
//		display.setTextColor(WHITE);
//		display.setCursor(60,15);
//		display.println(millis()/1000);
//		display.display();
//		lastMillis = currentMillis;
//
//	}

//}


void displayMenu0()
{
	display.clear();
	//display.inverse();
	display.drawString(0,0,"Main");
	display.setCursor(3, 1);
	//display.noInverse();
	display.println("Model 1");
	display.setCursor(3, 2);
	display.println("Model 2");
	display.setCursor(3, 3);
	display.println("Model 3");

}


void sendCurrentPsxState()
{
	// only send if enough time has been between the last send and now
	if (btLastSend + BT_MIN_SEND_INTERVAL_MILLIS < millis())
	{

		uint8_t values[PROT_ARRAY_LENGTH];

		// sticks
		//values[PROT_STICK_LX] = ps2x.Analog(PSS_LX);
		values[PROT_STICK_LX] = calculateStickValueWithDeadZoneAndTrim(PSS_LX);
		//values[PROT_STICK_LY] = ps2x.Analog(PSS_LY);
		values[PROT_STICK_LY] = calculateStickValueWithDeadZoneAndTrim(PSS_LY);
		// values[PROT_STICK_RX] = ps2x.Analog(PSS_RX);
		values[PROT_STICK_RX] = calculateStickValueWithDeadZoneAndTrim(PSS_RX);
		// values[PROT_STICK_RY] = ps2x.Analog(PSS_RY);
		values[PROT_STICK_RY] = calculateStickValueWithDeadZoneAndTrim(PSS_RY);

		// top buttons
		values[PROT_BTN_SQUARE] = ps2x.Analog(PSAB_SQUARE);
		values[PROT_BTN_TRIANGLE] = ps2x.Analog(PSAB_TRIANGLE);
		values[PROT_BTN_CROSS] = ps2x.Analog(PSAB_CROSS);
		values[PROT_BTN_CIRCLE] = ps2x.Analog(PSAB_CIRCLE);

		// back buttons
		values[PROT_BTN_L1] = ps2x.Analog(PSAB_L1);
		values[PROT_BTN_L2] = ps2x.Analog(PSAB_L2);
		values[PROT_BTN_R1] = ps2x.Analog(PSAB_R1);
		values[PROT_BTN_R2] = ps2x.Analog(PSAB_R2);

		// pad
		values[PROT_UP] = ps2x.Analog(PSAB_PAD_UP);
		values[PROT_DONW] = ps2x.Analog(PSAB_PAD_DOWN);
		values[PROT_LEFT] = ps2x.Analog(PSAB_PAD_LEFT);
		values[PROT_RIGHT] = ps2x.Analog(PSAB_PAD_RIGHT);

		// digital buttons
		values[PROT_DIGITAL_BUTTONS]=0;
		bitWrite(values[PROT_DIGITAL_BUTTONS], PROT_DIGITAL_START, ps2x.Button(PSB_START));
		bitWrite(values[PROT_DIGITAL_BUTTONS], PROT_DIGITAL_SELECT, ps2x.Button(PSB_SELECT));
		bitWrite(values[PROT_DIGITAL_BUTTONS], PROT_DIGITAL_STICK_BTN_L, ps2x.Button(PSB_L3));
		bitWrite(values[PROT_DIGITAL_BUTTONS], PROT_DIGITAL_STICK_BTN_R, ps2x.Button(PSB_R3));



		//Serial.println("sending");
		// calculate crc
		uint8_t myCRC = CRC8.smbus(values, sizeof(values));

		// send data via HM10
		bleSerial.write(values, sizeof(values));
		// send crc
		bleSerial.write(myCRC);
		// send newline (only for hterm debugging. will be removed later)
//		bleSerial.write("\r\n");
		// remember time of last send
		btLastSend = millis();
//		Serial.write(values, sizeof(values));
//		Serial.write(myCRC);
//		Serial.write("\r\n");

	}
}

/**
 * adds deadZone and trim to a stick
 */
uint8_t calculateStickValueWithDeadZoneAndTrim (byte stickIdentifier)
{


	signed int value = ps2x.Analog(stickIdentifier);

//	Serial.print("ID: ");
//	Serial.print(stickIdentifier);
//	Serial.print(" value: ");
//	Serial.print(value);

	char trim = 0;
	switch (stickIdentifier)
	{
		case PSS_LX:
		{
			trim = sticksTrim[TRIM_LX];
			break;
		}
		case PSS_LY:
		{
			trim = sticksTrim[TRIM_LY];
			break;
		}
		case PSS_RX:
		{
			trim = sticksTrim[TRIM_RX];
			break;
		}
		case PSS_RY:
		{
			trim = sticksTrim[TRIM_RY];
			break;
		}
	}

	// apply deadZone
	if ( (value > 127 - DEAD_ZONE) && (value < 127 +  DEAD_ZONE))
	{
		value = 127;
	}
	else if (value > 127)
	{
		value = map (value, 127 + DEAD_ZONE, 255, 128, 255);
	}
	else if (value < 127)
	{
		value = map (value, 0, 127 - DEAD_ZONE,0, 126);
	}
	// apply trim AFTER deadZone
	value = value + trim;

	// cut to 0 or 255 if
	if (value < 0)
	{
		value = 0;
	}
	else if (value > 255)
	{
		value = 255;
	}


//	Serial.print(" deadZone: ");
//	Serial.print((int) DEAD_ZONE);
//	Serial.print(" trim: ");
//	Serial.print(trim, DEC);
//	Serial.print(" result : ");
//	Serial.println(value);

	// apply reverse
	if (stickReverse[TRIM_LX] && stickIdentifier == PSS_LX)
	{
		value = map(value,0,255,255,0);
	}
	else if (stickReverse[TRIM_RX] && stickIdentifier == PSS_RX)
	{
		value = map(value,0,255,255,0);
	}
	else if (stickReverse[TRIM_LY] && stickIdentifier == PSS_LY)
	{
		value = map(value,0,255,255,0);
	}
	else if (stickReverse[TRIM_RY] && stickIdentifier == PSS_RY)
	{
		value = map(value,0,255,255,0);
	}

	return (uint8_t) value;
}


/**
 * currently hardcoded:
 * - left / right trims RX
 * - up / down trims LY
 */
void processTrimChange()
{
	if (ps2x.Button(PSB_PAD_RIGHT) && sticksTrim[TRIM_RX] < 127)
	{
		sticksTrim[TRIM_RX] = sticksTrim[TRIM_RX] +1;
	}
	else if (ps2x.Button(PSB_PAD_LEFT) && sticksTrim[TRIM_RX] > -126)
	{
		sticksTrim[TRIM_RX] = sticksTrim[TRIM_RX] -1;
	}

	if (ps2x.Button(PSB_PAD_UP) && sticksTrim[TRIM_LY] < 127)
	{
		sticksTrim[TRIM_LY] = sticksTrim[TRIM_LY] +1;
	}
	else if (ps2x.Button(PSB_PAD_DOWN) && sticksTrim[TRIM_LY] > -126)
	{
		sticksTrim[TRIM_LY] = sticksTrim[TRIM_LY] -1;
	}

}

void loop() {


	/* You must Read Gamepad to get new values and set vibration values
	 ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
	 if you don't enable the rumble, use ps2x.read_gamepad(); with no values
	 You should call this at least once a second
	 */
	if (error > 0) {	//skip loop if no controller found
		Serial.print(F("have error: "));
		Serial.println(error);
		blinkErrorCode();
		// displayError();
	}
//	else if (!isModelChoosen)
//	{
//		displayMenu0();
//		delay(1000);
//
//	}
	else
	{
		ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
		// check if we are trimming now
		processTrimChange();

		// send the values
		sendCurrentPsxState();
		//printDebug();

//		display.clearLine(0);
//		display.setCursor(0, 0);
		//signed int xPos= ps2x.Analog(PSS_LX);

//		display.print((signed int)ps2x.Analog(PSS_LX));

//		display.setCursor(5, 0);
//		display.print((signed int)ps2x.Analog(PSS_LY));
//
//		display.clearLine(2);
//		display.setCursor(0, 2);
//		display.print((signed int)ps2x.Analog(PSS_RX));
//		display.setCursor(5, 2);
//		display.print((signed int)ps2x.Analog(PSS_RY));

//		if (ps2x.Button(PSB_START))  //will be TRUE as long as button is pressed
//		{
//			Serial.println("Start is being held");
//		}
//		if (ps2x.Button(PSB_SELECT))
//		{
//			Serial.println("Select is being held");
//
//		}
		if (ps2x.Button(PSB_PAD_UP))
		{ //will be TRUE as long as button is pressed
			Serial.print(F("Up held this hard: "));
			Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
		}
		if (ps2x.Button(PSB_PAD_RIGHT))
		{
			Serial.print(F("Right held this hard: "));
			Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
		}
		if (ps2x.Button(PSB_PAD_LEFT))
		{
			Serial.print(F("LEFT held this hard: "));
			Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
		}
		if (ps2x.Button(PSB_PAD_DOWN))
		{
			Serial.print(F("DOWN held this hard: "));
			Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
		}

		if (ps2x.Button(PSAB_CIRCLE))
		{
			Serial.print(F("circle held this hard: "));
			Serial.println(ps2x.Analog(PSAB_CIRCLE), DEC);
		}

		if (ps2x.Button(PSB_L2))
		{
//			Serial.print("L2 pressed: ");
//			Serial.println(ps2x.Analog(PSAB_L2));
			digitalWrite(13, HIGH);
		}
		else
		{
			digitalWrite(13, LOW);

		}
//		vibrate = ps2x.Analog(PSAB_CROSS); //this will set the large motor vibrate speed based on how hard you press the blue (X) button
//		if (ps2x.NewButtonState()) { //will be TRUE if any button changes state (on to off, or off to on)
//			if (ps2x.Button(PSB_L3))
//				Serial.println("L3 pressed");
//			if (ps2x.Button(PSB_R3))
//				Serial.println("R3 pressed");
//
//			if (ps2x.Button(PSB_R2))
//				Serial.println("R2 pressed");
//
//			if (ps2x.Button(PSB_TRIANGLE))
//				Serial.println("Triangle pressed");
//		}

//		if (ps2x.ButtonPressed(PSB_CIRCLE)) //will be TRUE if button was JUST pressed
//			Serial.println("Circle just pressed");
//		if (ps2x.NewButtonState(PSB_CROSS)) //will be TRUE if button was JUST pressed OR released
//			Serial.println("X just changed");
//		if (ps2x.ButtonReleased(PSB_SQUARE)) //will be TRUE if button was JUST released
//			Serial.println("Square just released");

//		if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) { //print stick values if either is TRUE
//			Serial.print("Stick Values:");
//			Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
//			Serial.print(",");
//			Serial.print(ps2x.Analog(PSS_LX), DEC);
//			Serial.print(",");
//			Serial.print(ps2x.Analog(PSS_RY), DEC);
//			Serial.print(",");
//			Serial.println(ps2x.Analog(PSS_RX), DEC);
//		}
	}

//	counterDisplay();
	#ifdef DEBUG_FREE_MEMORY
		printSerialFreeMemory();
	#endif
	// delay(50);

	// update voltage level on display
	displayDebug();

}
