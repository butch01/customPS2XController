#include <arduino.h>
#include <PS2X_lib.h>  //for v1.6
#include <U8x8lib.h>
#include <MemoryFree.h>
#include <SoftwareSerial.h>
#include <FastCRC.h>
#include <LegoIr.h>




/* define debug stuff */
#define DEBUG_FREE_MEMORY 0
#define DEBUG_BLE_AT 1
#define DEBUG_BLE_PACKET_NUMBER 1
#define DEBUG_BLE_SEND 1
#define DEBUG 1
#define DEBUG_SERIAL Serial
#define DEBUG_BAUD 57600



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
#define BLE_DISCOVER_ENTRY_COUNT 4
#define BLE_DISCOVER_NAME_LENGTH 14
#define BLE_DISCOVER_ADDRESS_LENGTH 12
#define BLE_AT_DELAY 30

#define BLE_CONNECT_ERROR_CODE_OK 0
#define BLE_CONNECT_ERROR_CODE_ERROR 1
#define BLE_CONNECT_ERROR_CODE_FAIL 2

uint8_t blePacketNumber=0; // packet number for BLE


/* LEGO */
LegoIr pf;
#define LEGO_IR_PIN 9


/* SERIAL */
#define SERIAL_BAUD 57600



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

// cursor position by user
uint8_t menuCursorUserY = 2;
bool	isCursorChanged = true;
#define DISPLAY_NUMBER_OF_LINES 8





uint8_t menuAction = 0;
#define MENU_ACTION_NONE 0
#define MENU_ACTION_START 1
#define MENU_ACTION_CANCEL 2
#define MENU_ACTION_SELECT 3
#define MENU_ACTION_SELECT_ALT 4

// uint8_t menuSubId=0;

// define MENU_IDS
#define MENU_ID_NONE 0
#define MENU_ID_MAIN 1
#define MENU_ID_PROTOCOL_CHOOSER 2
#define MENU_ID_BLE_SCAN 3
#define MENU_ID_LEGO 4

uint8_t menuIdToShow 		= MENU_ID_MAIN; // default menu
bool 	isMenuMode 			= true;
uint8_t menuNumberOfEntries = 0;


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
//#define BT_MIN_SEND_INTERVAL_MILLIS 50
#define BT_MIN_SEND_INTERVAL_MILLIS 60
//#define BT_MIN_SEND_INTERVAL_MILLIS 300
unsigned long btLastSend=0;

// CRC
FastCRC8 CRC8;

// stick adjustment
#define DEAD_ZONE 25

#define STICK_LX 0
#define STICK_LY 1
#define STICK_RX 2
#define STICK_RY 3
char sticksTrim[] = {0,0,0,0};
float sticksExpo[] = {2.0,1.0,2.0,1.0};

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


/**
 * logs if DEBUG_BLE_AT is defined
 */
void logBLEAT (String logString)
{
	#ifdef DEBUG_BLE_AT
	if (Serial.availableForWrite())
	{
		Serial.print (logString);
	}
	#endif
}


/**
 * logs if DEBUG is defined
 */
void debug (String logString)
{
	#ifdef DEBUG
	if (Serial.availableForWrite())
	{
		Serial.print (logString);
	}
	#endif
}

void debug (int integer)
{
	#ifdef DEBUG
	if (Serial.availableForWrite())
	{
		Serial.print (integer);
	}
	#endif
}


/**
 * update voltage on display
 */
void displayVoltage()
{
	if (millis() - lastVoltagePrinted > (unsigned long) VOLTAGE_READ_INTERVAL_MS)
	{
		display.setCursor(11, 1);
		display.print(readVoltage(VOLTAGE_PIN_RAW, (float) VOLTAGE_DIVIDE_RESISTOR_1, (float) VOLTAGE_DIVIDE_RESISTOR_2));
		lastVoltagePrinted = millis();
	}
}

void displayDebug()
{


}

void displayWriteHeadline(char *headline)
{
	display.clearLine(0);
	display.clearLine(1);
	display.setCursor(0, 0);
	display.print(headline);
}

void displayWriteEntry(uint8_t lineNumber, char* stringToWrite)
{
	display.clearLine(2+lineNumber);
	display.setCursor(3, 2+lineNumber);
	display.print(stringToWrite);
}

void setup() {

	DEBUG_SERIAL.begin(DEBUG_BAUD);
	DEBUG_SERIAL.println(F("starting"));

	stickReverse[STICK_LY]=true;
	display.begin();
	delay (BLE_AT_DELAY);
	display.setPowerSave(0);
	display.clear();
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
			DEBUG_SERIAL.print(F("Found Controller, configured successful "));
			DEBUG_SERIAL.print(F("pressures = "));
			if (pressures)
				DEBUG_SERIAL.println(F("true "));
			else
				DEBUG_SERIAL.println(F("false"));
			DEBUG_SERIAL.print("rumble = ");
			if (rumble)
				DEBUG_SERIAL.println("true)");
			else {
				DEBUG_SERIAL.println("false");
				DEBUG_SERIAL.println(F("Try out all the buttons, X will vibrate the controller, faster as you press harder;"));
				DEBUG_SERIAL.println(
						"holding L1 or R1 will print out the analog stick values.");
				DEBUG_SERIAL.println(F("Note: Go to www.billporter.info for updates and to report bugs."));
			}
		}
		else if (error == 1)
		{
			DEBUG_SERIAL.println(F("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips"));
		}
		else if (error == 2)
		{
			DEBUG_SERIAL.println(F("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips"));
		}

		else if (error == 3)
		{
			DEBUG_SERIAL.println(F("Controller refusing to enter Pressures mode, may not support it. "));
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

	DEBUG_SERIAL.print(ps2x.Analog(PSS_LX), DEC);
	DEBUG_SERIAL.print(" ");
	DEBUG_SERIAL.print(ps2x.Analog(PSS_LY), DEC);
	DEBUG_SERIAL.print(" ");
	DEBUG_SERIAL.print(ps2x.Analog(PSS_RX), DEC);
	DEBUG_SERIAL.print(" ");
	DEBUG_SERIAL.print(ps2x.Analog(PSS_RY), DEC);
	DEBUG_SERIAL.print("\n");


	//delay(2000);
	bleSerial.begin(BLE_BAUDRATE);
	delay(BLE_AT_DELAY);

	DEBUG_SERIAL.println(F("AT"));
	bleSerial.println(F("AT"));
	delay(BLE_AT_DELAY);
	bleReadSerialLogOnly();

	bleSendATWakeUp();
	delay(BLE_AT_DELAY);
	bleReadSerialLogOnly();

	DEBUG_SERIAL.println(F("AT+ROLE1"));
	bleSerial.println(F("AT+ROLE1"));
	delay(BLE_AT_DELAY);
	bleReadSerialLogOnly();

//	bleSerial.println("AT+BAUD1");
//	delay(200);
//	bleReadSerial();

	DEBUG_SERIAL.println(F("AT+POWE2"));
	bleSerial.println(F("AT+POWE2"));
	delay(BLE_AT_DELAY);
	bleReadSerialLogOnly();

	DEBUG_SERIAL.println(F("AT+NOTI1"));
	bleSerial.println(F("AT+NOTI1"));
	delay(BLE_AT_DELAY);
	bleReadSerialLogOnly();

	DEBUG_SERIAL.println(F("AT+RESET"));
	bleSerial.println(F("AT+RESET"));
	delay(BLE_AT_DELAY);
	bleReadSerialLogOnly();





	DEBUG_SERIAL.println(F("AT+NAMESENDER"));
	bleSerial.println(F("AT+NAMESENDER"));
	delay(BLE_AT_DELAY);
	bleReadSerialLogOnly();


//	bleSerial.println("AT+CONA20C38FEF6C33");
//	delay(2000);
	// bleSerial.println("AT+CONN1");

	//bleScan();

	DEBUG_SERIAL.println(F("setup end"));
}


/**
 *  returns selected element (cursor position minus 2 (number of headlines)
 */
uint8_t getSelectedElementId()
{
	return menuCursorUserY -2;
}

/**
 * resets the menuAction.
 */
void menuActionReset()
{
	menuAction = MENU_ACTION_NONE;
}

void resetMenuCursor()
{
	menuCursorUserY = 2;
}

/**
 * main menu
 */
void menuMain()
{
	debug (F("menuMain in\n"));
	display.clear();
	menuNumberOfEntries = 1;
	resetMenuCursor();
	displayWriteHeadline("Main");
	displayWriteEntry(0, "Protocol");
	bool isBreakWhile = false;
	while (!isBreakWhile)
	{

		// process menu actions. Will return after we select, start, etc. is pressed.
		processMenuAction();

		if (menuAction == MENU_ACTION_START || menuAction == MENU_ACTION_CANCEL || menuAction == MENU_ACTION_SELECT)
		{
			isBreakWhile = true;
		}
	}


	// load submenu if seleced
	if ((menuAction == MENU_ACTION_SELECT) && (getSelectedElementId() == 0))
	{
		debug(F("calling starting protocolChooser\n"));
		// load submenu ProtocolChooser
		menuIdToShow = MENU_ID_PROTOCOL_CHOOSER;

	}
	else if (menuAction == MENU_ACTION_START || menuAction == MENU_ACTION_CANCEL)
	{
		debug(F("callingMenuNone\n"));
		// clear display, end menu mode
		menuIdToShow = MENU_ID_NONE;
	}
	menuActionReset();
	debug (F("menuMain out\n"));
}

/**
 * menu for choosing protocol
 */
void menuProtocolChooser()
{
	display.clear();
	menuNumberOfEntries = 2;
	resetMenuCursor();
	displayWriteHeadline("Protocol");
	displayWriteEntry(0, "LEGO");
	displayWriteEntry(1, "BLE");
	bool isBreakWhile = false;
	while (!isBreakWhile)
	{
		processMenuAction();
		debug (menuAction);
		if (menuAction == MENU_ACTION_CANCEL || menuAction == MENU_ACTION_SELECT || menuAction == MENU_ACTION_START)
		{
			isBreakWhile = true;
		}
	}


	if ((menuAction == MENU_ACTION_SELECT) && (getSelectedElementId() == 0))
	{
		// LEGO selected
		// not implemented yet

	}
	else if ((menuAction == MENU_ACTION_SELECT) && (getSelectedElementId() == 1))
	{
		// BLE selected
		// initiate BLE scan
		menuIdToShow = MENU_ID_BLE_SCAN;
	}
	else if (menuAction == MENU_ACTION_CANCEL)
	{
		// show upper level (main)
		menuIdToShow = MENU_ID_MAIN;
	}
	else if (menuAction == MENU_ACTION_START)
	{
		// end menu
		menuIdToShow = MENU_ID_NONE;
	}
	menuActionReset();

}

/**
 * prints the menu cursor
 */
void displayPrintMenuCursor ()
{
	for (uint8_t i=2; i< DISPLAY_NUMBER_OF_LINES; i++)
	{
		display.setCursor(0, i);
		if (i != menuCursorUserY)
		{
			display.print(' ');
		}
		else
		{
			display.print('>');
		}
	}
}


/**
 * menu for LEGO choose the Channel
 */
void menuLego()
{
	display.clear();
	resetMenuCursor();
	menuNumberOfEntries=4;
	displayWriteHeadline("LEGO");
	displayWriteEntry(0, "CH 1");
	displayWriteEntry(1, "CH 2");
	displayWriteEntry(2, "CH 3");
	displayWriteEntry(3, "CH 4");

	bool isBreakWhile = false;
		while (!isBreakWhile)
		{

			// process menu actions. Will return after we select, start, etc. is pressed.
			processMenuAction();

			if (menuAction == MENU_ACTION_START || menuAction == MENU_ACTION_CANCEL || menuAction == MENU_ACTION_SELECT)
			{
				isBreakWhile = true;
			}
		}


		// load submenu if seleced
		if (menuAction == MENU_ACTION_SELECT)
		{
			// initialize lego with the selected channel
			pf.begin(LEGO_IR_PIN, getSelectedElementId());
			menuIdToShow = MENU_ID_PROTOCOL_CHOOSER;

		}
		else if (menuAction == MENU_ACTION_START || menuAction == MENU_ACTION_CANCEL)
		{
			debug(F("callingMenuNone\n"));
			// clear display, end menu mode
			menuIdToShow = MENU_ID_NONE;
		}
		menuActionReset();
		debug (F("menuMain out\n"));

}


/**
 * menu for BLE scan
 */
void menuBleScan()
{
	display.clear();
	resetMenuCursor();
	displayWriteHeadline("BLE SCAN");
	menuActionBleScan();
}


/**
 * advanced action for BLE Scan menu
 */
void menuActionBleScan ()
{
	//debug (F("in blescan"));


	char discoveryNames[BLE_DISCOVER_ENTRY_COUNT][BLE_DISCOVER_NAME_LENGTH];
	char discoveryAddress[BLE_DISCOVER_ENTRY_COUNT][BLE_DISCOVER_ADDRESS_LENGTH];
	int  discoveryRSSI[BLE_DISCOVER_ENTRY_COUNT];

	//set results all empty
	for (int entry=0; entry < BLE_DISCOVER_ENTRY_COUNT; entry ++)
	{
		memset(discoveryNames, 0, sizeof(discoveryNames));
		memset(discoveryAddress, 0, sizeof(discoveryAddress));
		memset(discoveryRSSI, 0, sizeof(discoveryRSSI));
	}

	bleSerial.println(F("AT+DISC?"));

	// we save our response line here
	String responseLine = "";
	responseLine.reserve(64);

	bool isResponseCompletelyRead = false;
	int index = -1; // starting at -1. Will increment on first match if AT+DIS0
	int lineNumber=0;
	// read as long as response is not completely read
	while (!isResponseCompletelyRead)
	{
		//printSerialFreeMemory();
		if (bleSerial.available() >0)
		{
			// we have data in serial waiting
			responseLine = bleSerial.readStringUntil('\r');
			Serial.print(F("lineNumber: "));
			Serial.print(lineNumber);

			Serial.println(responseLine);
			if (responseLine.indexOf(F("OK+DIS0")) != -1)
			{
				// increment index first
				index++;

				// we have a line containing the address
				Serial.println(responseLine.substring(responseLine.indexOf(":")+1, responseLine.indexOf(":") +1 + 12));
				responseLine.substring(responseLine.indexOf(":")+1, responseLine.indexOf(":") +1 + 12).toCharArray(discoveryAddress[index], BLE_DISCOVER_ADDRESS_LENGTH+1);


				// also check if RSSI is available, save this too.
				if (int a= responseLine.indexOf(F("RSSI:")) != -1)
				{
					discoveryRSSI[index] = responseLine.substring(a+5).toInt();
				}

			}
			// search for Name starting with MYRC
			else if (responseLine.indexOf(F("OK+NAME")) != -1)
			{
				// we have a name
				Serial.println(responseLine.substring(responseLine.indexOf(":")+1));
				responseLine.substring(responseLine.indexOf(":")+1).toCharArray(discoveryNames[index], BLE_DISCOVER_NAME_LENGTH, 0);
			}
			// check for end sequence
			else if (responseLine.indexOf(F("OK+DISCE"))!=-1)
			{
				isResponseCompletelyRead=true;
			}
			lineNumber++;


		}

	}

	debug(F("names: "));
	debug(discoveryNames[0]);
	debug(discoveryNames[1]);
	debug(discoveryNames[2]);

	debug(discoveryAddress[0]);
	debug(discoveryAddress[1]);
	debug(discoveryAddress[2]);

	debug(F("names end "));
	debug(F("index: "));
	debug(String(index));
	debug(F("\n"));


	// display names on display
	for (uint8_t i=0; i<= index; i++)
	{
		displayWriteEntry(i, discoveryNames[i]);
		Serial.println(discoveryNames[i]);
	}

	bool isBreakWhile=false;

	// stay in loop until an entry is selected
	while (!isBreakWhile)
	{
		processMenuAction();
		menuNumberOfEntries = index;

		if (menuAction == MENU_ACTION_CANCEL || menuAction == MENU_ACTION_SELECT || menuAction == MENU_ACTION_START)
		{
			isBreakWhile = true;
		}
	}



	if (menuAction == MENU_ACTION_SELECT)
	{
		// we want to connect to a device

		// check if we selected a valid BLE device. If so, connect to it
		// if ((menuCursorUserY -2) <= index)
		//{
		// we have a valid match
		//uint8_t connStatus = bleConnect (discoveryAddress[menuCursorUserY-2]);
		uint8_t connStatus = bleConnect (discoveryAddress[getSelectedElementId()]);
		display.clear();

		switch (connStatus)
		{
			case BLE_CONNECT_ERROR_CODE_ERROR:
				displayWriteEntry(4, "ERROR!");
				break;

			case BLE_CONNECT_ERROR_CODE_FAIL:
				displayWriteEntry(4, "FAILED");
				break;

			case BLE_CONNECT_ERROR_CODE_OK:
				displayWriteEntry(4, "CONNECTED");
				isMenuMode = false;
				break;

		}

			//}

	}
	else if (menuAction == MENU_ACTION_CANCEL)
	{

		// return to upper menu level (protocol chooser)
		menuIdToShow = MENU_ID_PROTOCOL_CHOOSER;
	}
	else if (menuAction == MENU_ACTION_START)
	{
		// end menu
		menuIdToShow = MENU_ID_NONE;
	}
	menuActionReset();
	printSerialFreeMemory();
	debug(F("out of blescan"));
}


/**
 * calls a menu function by an id (menuIdToShow)
 */
void menuRouter()
{
	debug (F("menuRouter in\nMenuIdToShow: "));
	debug (menuIdToShow);
	debug (F("\n"));

	switch (menuIdToShow)
	{
		case MENU_ID_NONE:
			isMenuMode = false;
			display.clear();
			break;

		case MENU_ID_MAIN:
			menuMain();
			break;

		case MENU_ID_BLE_SCAN:
			menuBleScan();
			break;

		case MENU_ID_PROTOCOL_CHOOSER:
			menuProtocolChooser();
			break;

		case MENU_ID_LEGO:
			menuLego();
			break;

	}
	debug (F("menuRouter out\n"));
}


/**
 * send at command to connect to a specific address
 * returns the status
 * status codes: 0 - OK, 1 - Error, 2 - Failed
 */
uint8_t bleConnect(char *deviceAddress)
{


	uint8_t returnCode = 0;

//	char command[18];
//	memset(command, 0, sizeof(command));
//	strcpy(command, strcat("AT+CON",deviceAddress));
	String command = "AT+CON";
	command.concat(deviceAddress);
	Serial.print("command: ");
	Serial.println(command);

	logBLEAT(F("in function bleConnect\n"));
	logBLEAT(F("Address: "));
	logBLEAT(deviceAddress);
	logBLEAT("\n");
	logBLEAT(F("command:\n"));
	logBLEAT(command);

	bleSerial.println(command);

	bool isBreakWhile=false;

	// create response string
	String response;
	response.reserve(64);
	Serial.println(F("Response: \n"));

	while (!isBreakWhile)
	{
		Serial.println(bleSerial.available());
		if (bleSerial.available() >0)
		{
			// bleSerial.readBytesUntil('\r', response, sizeof(response));
			// Serial.println(response);
			response = (bleSerial.readStringUntil('\r'));
			response.trim();


			Serial.println(response);

			if (response.equals(F("OK+CONNE")))
			{
				returnCode = BLE_CONNECT_ERROR_CODE_ERROR;
				isBreakWhile = true;
			}
			else if (response.equals(F("OK+CONNF")))
			{
				returnCode = BLE_CONNECT_ERROR_CODE_FAIL;
				isBreakWhile = true;
			}
			else if (response.equals(F("OK+CONN")))
			{
				returnCode = BLE_CONNECT_ERROR_CODE_OK;
				isBreakWhile = true;
			}

			logBLEAT(F("BLE-Connect-Response: "));
			logBLEAT(response);
			logBLEAT("\n");

		}

	}
	return returnCode;
}


/* send long text for wakeup via AT*/
void bleSendATWakeUp ()
{
	bleSerial.println(F("sksnlskndvlknslkvnslvknklnlnlkndabcvsidbvsksnlskndvlknslkvnslvknklnlnlkndabcvsidbvsksnlskndvlknslkvnslvknklnlnlkndabcvsidbvsksnlskndvlknslkvnslvknklnlnlkndabcvsidbvAT"));
	bleSerial.println(F("AT"));
}


/**
 * ONLY logs the BLE Serial
 */
void bleReadSerialLogOnly()
{

	if (bleSerial.available() > 0)
	{
		logBLEAT(bleSerial.readString());
	}
}


void printSerialFreeMemory()
{
	#ifdef DEBUG_FREE_MEMORY
		if (millis() - (unsigned long) DEBUG_PRINT_FREE_MEM_TIMER > lastMemoryStatsMillis)
		{
			Serial.print(millis());
			Serial.print(F(" - freeMemory()="));
			Serial.println(freeMemory());
			lastMemoryStatsMillis = millis();
		}
	#endif
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
		// increase the packet number for this BLE packet
		blePacketNumber++;


		// packetnumber will go from 1 to 255
		if (blePacketNumber == 0)
		{
			blePacketNumber++;
		}

		uint8_t values[PROT_ARRAY_LENGTH];

		// sticks
		//values[PROT_STICK_LX] = ps2x.Analog(PSS_LX);
		//values[PROT_STICK_LX] = calculateStickValueWithDeadZoneAndTrim(PSS_LX);
		values[PROT_STICK_LX] = applyExpoValue(calculateStickValueWithDeadZoneAndTrim(PSS_LX),PSS_LX);
		//values[PROT_STICK_LY] = ps2x.Analog(PSS_LY);
		//values[PROT_STICK_LY] = calculateStickValueWithDeadZoneAndTrim(PSS_LY);
		values[PROT_STICK_LY] = applyExpoValue(calculateStickValueWithDeadZoneAndTrim(PSS_LY),PSS_LY) ;
		// values[PROT_STICK_RX] = ps2x.Analog(PSS_RX);
		//values[PROT_STICK_RX] = calculateStickValueWithDeadZoneAndTrim(PSS_RX);
		values[PROT_STICK_RX] = applyExpoValue(calculateStickValueWithDeadZoneAndTrim(PSS_RX), PSS_RX);
		// values[PROT_STICK_RY] = ps2x.Analog(PSS_RY);
		values[PROT_STICK_RY] = applyExpoValue(calculateStickValueWithDeadZoneAndTrim(PSS_RY),PSS_RY) ;

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
//		values[PROT_DIGITAL_BUTTONS]=0;
//		bitWrite(values[PROT_DIGITAL_BUTTONS], PROT_DIGITAL_START, ps2x.Button(PSB_START));
//		bitWrite(values[PROT_DIGITAL_BUTTONS], PROT_DIGITAL_SELECT, ps2x.Button(PSB_SELECT));
//		bitWrite(values[PROT_DIGITAL_BUTTONS], PROT_DIGITAL_STICK_BTN_L, ps2x.Button(PSB_L3));
//		bitWrite(values[PROT_DIGITAL_BUTTONS], PROT_DIGITAL_STICK_BTN_R, ps2x.Button(PSB_R3));

		// debug dont send digital buttons. Use this as messageId
		values[PROT_DIGITAL_BUTTONS] = blePacketNumber;

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

		#if DEBUG_BLE_SEND == 1
			unsigned long currentTime = millis();
			DEBUG_SERIAL.print(currentTime - btLastSend);
			DEBUG_SERIAL.print(F(" - "));
			DEBUG_SERIAL.print(currentTime);
			DEBUG_SERIAL.print(F(" - "));

			for (uint8_t i=0; i< sizeof(values); i++)
			{
				DEBUG_SERIAL.print(values[i], HEX);
				//DEBUG_SERIAL.print(" ");
			}
			DEBUG_SERIAL.print(myCRC, HEX);
			DEBUG_SERIAL.print("\r\n");
		#endif
		btLastSend = millis();
	}
}

/**
 * readIncomingBLE
 */
void readIncomingBLE ()
{

	if (bleSerial.available() > 0)
	{

		//char responseString[32];
		//bleSerial.readBytes(responseString, sizeof responseString);
		String responseString;
		responseString = bleSerial.readString();



		// debug response to serial
		#if DEBUG_BLE_PACKET_NUMBER == 1
			DEBUG_SERIAL.print("ack=");
			DEBUG_SERIAL.println(responseString);
		#endif

	}
}

/**
 * adds deadZone and trim to a stick
 */
uint8_t calculateStickValueWithDeadZoneAndTrim (byte stickIdentifier)
{
	signed int value = ps2x.Analog(stickIdentifier);

	char trim = 0;
	switch (stickIdentifier)
	{
		case PSS_LX:
		{
			trim = sticksTrim[STICK_LX];
			break;
		}
		case PSS_LY:
		{
			trim = sticksTrim[STICK_LY];
			break;
		}
		case PSS_RX:
		{
			trim = sticksTrim[STICK_RX];
			break;
		}
		case PSS_RY:
		{
			trim = sticksTrim[STICK_RY];
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


	// apply reverse
	if (stickReverse[STICK_LX] && stickIdentifier == PSS_LX)
	{
		value = map(value,0,255,255,0);
	}
	else if (stickReverse[STICK_RX] && stickIdentifier == PSS_RX)
	{
		value = map(value,0,255,255,0);
	}
	else if (stickReverse[STICK_LY] && stickIdentifier == PSS_LY)
	{
		value = map(value,0,255,255,0);
	}
	else if (stickReverse[STICK_RY] && stickIdentifier == PSS_RY)
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
	if (ps2x.Button(PSB_PAD_RIGHT) && sticksTrim[STICK_RX] < 127)
	{
		sticksTrim[STICK_RX] = sticksTrim[STICK_RX] +1;
	}
	else if (ps2x.Button(PSB_PAD_LEFT) && sticksTrim[STICK_RX] > -126)
	{
		sticksTrim[STICK_RX] = sticksTrim[STICK_RX] -1;
	}

	if (ps2x.Button(PSB_PAD_UP) && sticksTrim[STICK_LY] < 127)
	{
		sticksTrim[STICK_LY] = sticksTrim[STICK_LY] +1;
	}
	else if (ps2x.Button(PSB_PAD_DOWN) && sticksTrim[STICK_LY] > -126)
	{
		sticksTrim[STICK_LY] = sticksTrim[STICK_LY] -1;
	}

}

/**
 * calculates new cursor position and update the drawing
 */
void processCursorNavigation()
{
	if (ps2x.ButtonPressed(PSB_PAD_UP))
	{
		if (menuCursorUserY > 2)
		{
			menuCursorUserY--;
			isCursorChanged=true;
		}
	}
	else if (ps2x.ButtonPressed(PSB_PAD_DOWN))
	{
		if ((menuCursorUserY < DISPLAY_NUMBER_OF_LINES -1 ) && (menuCursorUserY < 2+ menuNumberOfEntries - 1))
		{
			menuCursorUserY++;
			isCursorChanged=true;
		}
	}

	// update cursor position if cursor's position has been changed
	if (isCursorChanged)
	{
		displayPrintMenuCursor();
	}
}


/**
 * processes the buttons when you are in menu.
 * cross:   cancel (one level up)
 * square:  select / OK
 * circle:  alternative selection (like edit)
 * start:   close menu
 */
void processMenuAction()
{
	bool isBreakWhile = false;

	while (!isBreakWhile)
	{
		// update gamepad status
		ps2x.read_gamepad();

		// process cursor navigation (up down)
		processCursorNavigation();

		// process menu action buttons for select, cancel, etc.
		if (ps2x.ButtonPressed(PSB_CROSS))
		{
			menuAction = MENU_ACTION_CANCEL;
			isBreakWhile = true;
		}
		else if (ps2x.ButtonPressed(PSB_SQUARE))
		{
			menuAction = MENU_ACTION_SELECT;
			isBreakWhile = true;
		}
		else if (ps2x.ButtonPressed(PSB_CIRCLE))
		{
			menuAction = MENU_ACTION_SELECT_ALT;
			isBreakWhile = true;
		}
		else if (ps2x.ButtonPressed(PSB_START))
		{
			menuAction = MENU_ACTION_START;
			isBreakWhile = true;
		}
	}
}


/**
 * add expo value
 */
uint8_t applyExpoValue(uint8_t inputValue, uint8_t stickIdentifier)
{
	float expo=1;

	switch (stickIdentifier)
	{
		case PSS_LX:
		{
			expo = sticksExpo[STICK_LX];
			break;
		}
		case PSS_LY:
		{
			expo = sticksExpo[STICK_LY];
			break;
		}
		case PSS_RX:
		{
			expo = sticksExpo[STICK_RX];
			break;
		}
		case PSS_RY:
		{
			expo = sticksExpo[STICK_RY];
			break;
		}
	}

	if (stickIdentifier == PSS_RX)
	{
		debug (F("input="));
		debug (inputValue);
		debug (F("  "));
		debug(F("expo="));
		debug(expo);
		debug (F(" "));
	}

	// map is only working with long values, not with float. So we use range from 0 to 1000, which means 3 decimal values;
	float  inputValue1000=0;
	if (inputValue > 127)
	{
		 inputValue1000 = map(inputValue, 128, 255, 0, 1000);
	}
	else if  (inputValue < 127)
	{
		 inputValue1000 = map(inputValue, 126, 0, 0, 1000);
	}


	//float withExpo = ((inputValue1000/1000) ^ expo)*1000;
	float withExpo = pow(inputValue1000/1000, expo)*1000;

//	debug (F("with expo="));
//	debug (withExpo);

	if (inputValue > 127)
	{
		inputValue =  map(withExpo, 0,1000,128,255);
	}
	else if (inputValue < 127)
	{
		inputValue =  map(withExpo, 0,1000,126,0);
	}

	if (stickIdentifier == PSS_RX)
	{
		debug (F(" final"));
		debug (inputValue);
		debug (F(" - "));
		// debug (F("\n"));
	}
	return inputValue;

}





void loop()
{
	// update voltage on display
	displayVoltage();



	/* You must Read Gamepad to get new values and set vibration values
	 ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
	 if you don't enable the rumble, use ps2x.read_gamepad(); with no values
	 You should call this at least once a second
	 */

	ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
	if (ps2x.ButtonReleased(PSB_START))
	{
		// enable menuMode
		isMenuMode = true;
		menuIdToShow = MENU_ID_MAIN;
	}

	// check if we are in menu mode
	while (isMenuMode)
	{
		// call menuRouter
		menuRouter();
	}

		// read responses
		readIncomingBLE();

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


	//	counterDisplay();
//		#ifdef DEBUG_FREE_MEMORY
//			printSerialFreeMemory();
//		#endif
//		// delay(50);
//
//		// update voltage level on display
//		displayDebug();

}
