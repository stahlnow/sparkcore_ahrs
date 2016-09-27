#include "application.h"
#include "OSCMessage.h"
#include "OSCBundle.h"
#include "AHRS.h"

// Create AHRS object
AHRS* ahrs;

//SYSTEM_MODE(AUTOMATIC);


//----- OUTPUTS
int led1 = D6;
int led2 = D7;

//------ Encoder Variables
int encoderPinA = D2;
int encoderPinB = D3;
int pushButton = D4;

volatile int lastEncoded = 0;
volatile long encoderValue = 0;
long lastencoderValue = 0;
int lastMSB = 0;
int lastLSB = 0;


//----- REGISTERED OSC COMMANDS
char OscCmd_Encoder[9] = "/encoder";
char OscCmd_led[5] = "/led";							// 4 characters  + 1 for the "null-terminated string" -> '\0'
char OscCmd_TestSendMsg[13] = "/sendTestMsg";			// 12 characters + 1 for the "null-terminated string" -> '\0'
char OscCmd_TestSendBndl[14] = "/sendTestBndl";			// 13 characters + 1 for the "null-terminated string" -> '\0'
char OscCmd_SwitchToMessages[16] = "/manageMessages";	// 15 characters + 1 for the "null-terminated string" -> '\0'
char OscCmd_SwitchToBundles[15] = "/manageBundles";		// 14 characters + 1 for the "null-terminated string" -> '\0'


//----- IP ADRESSES
IPAddress computerIPAddress = IPAddress(10,40,10,105);	// put the IP address of your computer here 10.0.2.153
IPAddress coreIPAddress;
OSCMessage coreIPMessage("/coreip");


//----- PORTS
#define LOCALPORT  8888		// to send data to the Spark Core (from the computer)
#define REMOTEPORT 57121	// to send data to the computer (from here)


//----- MANAGING OSC MESSAGES OR OSC BUNDLES
int manageMessages = 0;
int manageBundles = 0;


//----- UDP + overloading the inappropriate UDP functions of the Spark Core (REQUIRED !)
class myUDP : public UDP {
	private :
	uint8_t myBuffer[512];
	int offset = 0;
	public :
	virtual int beginPacket(IPAddress ip, uint16_t port){
		offset = 0;
		return UDP::beginPacket(ip, port);
	};
	virtual int endPacket(){
		return UDP::write(myBuffer, offset);
	};
	virtual size_t write(uint8_t buffer) {
		write(&buffer, 1);
		return 1;
	}
	virtual size_t write(const uint8_t *buffer, size_t size) {
		memcpy(&myBuffer[offset], buffer, size);
		offset += size;
		return size;
	}
};

myUDP Udp;



void updateEncoder() {
	int MSB = digitalRead(encoderPinA); //MSB = most significant bit
	int LSB = digitalRead(encoderPinB); //LSB = least significant bit

	int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
	int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

	if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue++;
	if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue--;

	lastEncoded = encoded; //store this value for next time
}

//=========================================================================================
//=========================================================================================
void setup()
{

	// Initialize led pins as output for the two LEDs
	pinMode(led1, OUTPUT);
	pinMode(led2, OUTPUT);

	// Start UDP
	Udp.begin(LOCALPORT);

	// Get the IP address of the Spark Core and send it as an OSC Message
	coreIPAddress = WiFi.localIP();
	coreIPMessage.add(coreIPAddress[0]).add(coreIPAddress[1]).add(coreIPAddress[2]).add(coreIPAddress[3]);

	Udp.beginPacket(computerIPAddress, REMOTEPORT);
	coreIPMessage.send(Udp);
	Udp.endPacket();

	// First manage OSCMessages and not OSCBundles
	manageMessages = 1;

	// Initialize Encoder

	pinMode(encoderPinA, INPUT_PULLUP);
	pinMode(encoderPinB, INPUT_PULLUP);
	pinMode(pushButton, INPUT_PULLUP);

	// encoder pin on interrupt 0 (pin D2)
	attachInterrupt(D2, updateEncoder, CHANGE);
	// encoder pin on interrupt 1 (pin D3)
	attachInterrupt(D3, updateEncoder, CHANGE);

	digitalWrite(led1, HIGH);
	delay(1000);



    // Init serial output
    Serial.begin(57600);
    delay(2000);
    ahrs = new AHRS(OUTPUT__MODE_ANGLES, OUTPUT__FORMAT_TEXT, led1, false);
}

/*
//=========================================================================================
//===== TEST receiving an OSC Message made of the address "/led" and containing 2 ints : led number , led status (0=LOW , 1=HIGH)
//=========================================================================================
void setLEDStatus(OSCMessage &mess)
{
	if (mess.size() == 2 && mess.isInt(0) && mess.isInt(1)) {
		int thisLed = mess.getInt(0);
		int thisStatus = mess.getInt(1);

		if (thisLed == 0 && thisStatus == 0) {digitalWrite(led1,LOW);}
		else if (thisLed == 0 && thisStatus == 1) {digitalWrite(led1,HIGH);}
		else if (thisLed == 1 && thisStatus == 0) {digitalWrite(led2,LOW);}
		else if (thisLed == 1 && thisStatus == 1) {digitalWrite(led2,HIGH);}
		//	else {
		//		return;			// ERRORS : inappropriate led number or led status
		//	}
	}
	//	else {
	//		return;			// ERRORS : bad message size (!= 2) or the arguments are not ints
	//	}
}


void sendOSCEncoder(OSCMessage &mess)
{
	OSCMessage encMsg_toSend("/encoder");
	encMsg_toSend.add((int)encoderValue).add("ENCOOOOODER");

	Udp.beginPacket(computerIPAddress, REMOTEPORT);
	encMsg_toSend.send(Udp); // send the bytes
	Udp.endPacket();
	encMsg_toSend.empty(); // empty the message to free room for a new one

}

//=========================================================================================
//===== TEST sending an OSCBundle (called when the Spark Core receives "/sendTestBndl" as an OSC Message)
//=========================================================================================
void sendOSCTestBndl(OSCMessage &mess)
{
	OSCBundle testBndl_toSend;

	//OSCBundle's "add" returns the OSCMessage so the message's "add" can be composed together
	testBndl_toSend.add("/testbundle/msg1").add((float)5.6).add(250).add("hohoho").add(-2000);
	testBndl_toSend.add("/testbundle/msg2").add(124).add("hehehe").add((float)1.6).add(-150);

	testBndl_toSend.setTimetag((uint64_t)millis());
	// NB : millis() is "the number of milliseconds since the processor started up", not an appropriate NTP timetag !
	// setTimetag(oscTime()) DOESN'T WORK ON THE SPARK CORE (Oscuino has to rewrite the OSCTiming class)

	Udp.beginPacket(computerIPAddress, REMOTEPORT);
	testBndl_toSend.send(Udp); // send the bytes
	Udp.endPacket();
	testBndl_toSend.empty(); // empty the bundle to free room for a new one
}
*/

//=========================================================================================
void loop()
{
    // check ahrs for incoming serial commands
    ahrs->check_commands();
    ahrs->run();


    OSCMessage encMsg_toSend("/ahrs");
    encMsg_toSend.add((float)ahrs->getYaw()).add((float)ahrs->getPitch()).add((float)ahrs->getRoll());

    Udp.beginPacket(computerIPAddress, REMOTEPORT);
    encMsg_toSend.send(Udp); // send the bytes
    Udp.endPacket();
    encMsg_toSend.empty(); // empty the message to free room for a new one




    /*
	delay(10);

	// encoder
	OSCMessage encMsg_toSend("/encoder");
	encMsg_toSend.add("Encoder: ").add((int)encoderValue);

	Udp.beginPacket(computerIPAddress, REMOTEPORT);
	encMsg_toSend.send(Udp); // send the bytes
	Udp.endPacket();
	encMsg_toSend.empty(); // empty the message to free room for a new one
    */

}



