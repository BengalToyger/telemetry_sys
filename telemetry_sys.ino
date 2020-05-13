
#include <MPU9250.h>
#include <Adafruit_GPS.h>
#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>


#define GPSSerial Serial1
#define MS_CNT 15999


//STM32F042K6 Pins
#define RFM69_CS      7
#define RFM69_INT     2
#define RFM69_RST     4
#define LED           31

#define IMU_CS        49


// Where to send packets to!
#define DEST_ADDRESS   1
// change addresses for each client board, any number :)
#define MY_ADDRESS     2

#define RF69_FREQ 915.0
// GPS Driver
Adafruit_GPS GPS(&GPSSerial);

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);


MPU9250 IMU(SPI,IMU_CS);



int status;
char buf[RH_RF69_MAX_MESSAGE_LEN];
uint16_t packetnum = 0;  // packet counter, we increment per xmission
uint8_t sent = 0;
uint8_t len = 0;

void setup() {
	// serial to display data
	Serial.begin(115200);
	Serial.println("Online!");
	pinMode(RFM69_CS,OUTPUT);
	digitalWrite(RFM69_CS,HIGH);
	pinMode(IMU_CS,OUTPUT);
	digitalWrite(IMU_CS,HIGH);
	// start communication with IMU 
	gps_init();
	radio_init();
	imu_init();

}


ISR(TIMER1_COMPB_vect) {
	char c = GPS.read();
}


void loop() {
	char c = GPS.read();
	if (GPS.newNMEAreceived()) {
		GPS.parse(GPS.lastNMEA());
	}
	IMU.readSensor();
	send_data();
}

void setup_1ms_timer_int(){
	noInterrupts();           // disable all interrupts
	TCCR1A |= 0;
	TCCR1B |= (1 << WGM12)|(1 << CS10);

	OCR1A = MS_CNT;            
	TIMSK1 |= (1 << OCIE1A);   // enable timer compare match interrupt
	interrupts();             // enable all interrupts
}

void radio_init(){
	pinMode(LED, OUTPUT);     
 	pinMode(RFM69_RST, OUTPUT);
 	digitalWrite(RFM69_RST, LOW);
	digitalWrite(RFM69_RST, HIGH);
  	delay(10);
  	digitalWrite(RFM69_RST, LOW);
  	delay(10);

	if (!rf69_manager.init()) {
		Serial.println("RFM69 radio init failed");
		while (1);
	}
	Serial.println("RFM69 radio init OK!");
	// Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
	// No encryption
	if (!rf69.setFrequency(RF69_FREQ)) {
		Serial.println("setFrequency failed");
	}

	// If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
	// ishighpowermodule flag set like this:
	rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

	// The encryption key has to be the same as the one in the server
	uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
	    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
	rf69.setEncryptionKey(key);
	Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}

void gps_init(){

	// Initialize GPS with some settings
	GPS.begin(9600);
	// Recommended minimum data and fix data
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	// 1Hz updates
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
	// Ask for antenna data
	GPS.sendCommand(PGCMD_ANTENNA);

//	setup_1ms_timer_int();
}

void imu_init(){
	Serial.println("IMU");
	status = IMU.begin();
	if (status < 0) {
		Serial.println("IMU initialization unsuccessful");
		Serial.println("Check IMU wiring or try cycling power");
		Serial.print("Status: ");
		Serial.println(status);
		while(1) {}
	}
}

void send_data(){
	if (!(millis() % 1000) & !sent){
		sent = 1;
		Serial.println(GPS.latitudeDegrees, 4);
		Serial.println(GPS.longitudeDegrees, 4);
		len = sprintf(buf,"%f.4,%f.4,%d,%d,%d",
		GPS.latitudeDegrees,GPS.longitudeDegrees,
		(int16_t)IMU.getMagX_uT(),(int16_t)IMU.getMagY_uT(),
		(int16_t)IMU.getMagZ_uT());
		rf69_manager.sendtoWait((uint8_t*)buf,len,DEST_ADDRESS);		
	} else if (sent & (millis() % 1000)) {
		sent = 0;
	}
}
