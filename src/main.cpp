/*
*   Includes
*/
#include "math.h"
#include "register_definition.h"
#include "Energia.h"



/*
*   Macros
*/
#define lcdClear() lcdCmd(0x01)
#define lcdHome() lcdCmd(0x02)
#define lcdOn() lcdCmd(0x0C)
#define lcdClearLine(line) lcdPrint((char *)"                ",line)



/*
*   Function prototypes
*/
int distance(double c1[2], double c2[2]);
void convertCoords(char lat[10],char lon[10],double ddCoord[2]);
void lcdData(char data);
void lcdCmd(char cmd);
void lcdPrint(char *str , int line);
void lcdInit(void);
void swInit(void);
void redLedInit(void);
void uart2Init(void);
char uart2Rcv(void);
void parseSentence(char *prefix ,char *data, char sen[75]);
void getSpeed(char *sen,char speed[10]);
void getTime(char *sen,char time[6]);
void disableInt();
void enableInt();



/*
*   Global variables
*/
char gpsData[512];				//Holds the fetched GPS UART data
bool dataReady = 0;				//Flag, indicates that new GPS data have been fetched. 1: New data available, 0: No new data
int distSum = 0;				//Holds the total distance covered
double lastCoord[2] = {0};		//Holds the last known coordinates
double latAvg = 0;				//Holds the average latitude
double lonAvg = 0;				//Holds the average longitude
int setFlg = 0;					//Indicates that the averaging set is ready
int setCtr = 0;					//Averaging counter
int setSize = 3;				//Averaging set size, sets the number of points to take average of
unsigned int mode = 0;			//Holds the mode index. 0: Distance, 1: Displacement, 2: Coords, 3: Speed, 4: Atomic Clock
double currentCoord[2] = {0};	//Holds the current coordinates
double originCoord[2] = {0};	//Holds the origin coordinates, used by displacement mode



/*
*   Strings
*/
char distStr[] = "Distance:";
char distNoSig[] = "Distance:[!]";
char dispStr[] = "Displacement:";
char dispNoSig[] = "Displacement:[!]";
char coordStr[] = "Coordinates";
char speedStr[] = "Speed:";
char speedNoSig[] = "Speed:[!]";
char timeStr[] = "Atomic Clock:";
char timeNoSig[] = "Atomic Clock:[!]";
char noGps[] = "Waiting for GPS";



/*
 * Function:  distance
 * --------------------
 * Computes the distance between two coordinates by using the following formula (Haversine):
 *    Distance = (12742 * asin(0.5 - cos((c2[0]-c1[0])*p)/2 + cos(c1[0]*p) * cos(c2[0]*p) * (1-cos((c2[1]-c1[1])*p))/2)))*1000
 *
 *  c1: First coordinate.
 *  c2: Second coordinate.
 *
 *  returns: The calculated distance between the two coordinates (in meters) rounded
 */
int distance(double c1[2], double c2[2])
{
	float pi = 3.141592654;
	float p = pi/180;
	float a = 0.5 - cos((c2[0]-c1[0])*p)/2 + cos(c1[0]*p) * cos(c2[0]*p) * (1-cos((c2[1]-c1[1])*p))/2;
	return (int)(round((12742 * asin(sqrt(a)))*1000));
}



/*
 * Function:  convertCoords
 * --------------------
 * Convert coordinates DDM format (String) to DD by:
 *    1- Converts the DDM string to a double value.
 *    2- Seperate the degrees part and the seconds part.
 *    3- Converts the degrees and seconds into decimal degrees.
 *
 *  lat: Latitude.
 *  lon: Longitude.
 * 	ddCoord: The resultant DD coordinates.
 *
 *  returns: By reference, converted coordinates in DD.
 */
void convertCoords(char lat[10],char lon[10],double ddCoord[2])
{
	double degLat = 0;
	double degLon = 0;
	double secLat = 0;
	double secLon = 0;

	double latd = strtod(lat,NULL);
	double lond = strtod(lon,NULL);

	degLat = latd/100;
	degLon = lond/100;
	secLat = latd - ((int)degLat)*100;
	secLon = lond - ((int)degLon)*100;

	ddCoord[0] = (int)degLat+(secLat/60);
	ddCoord[1] = (int)degLon+(secLon/60);
}



/*
 * Function:  lcdData
 * --------------------
 * Sends data (character) to the LCD module in 4bit mode (nibble by nibble).
 *
 *  data: Data to send to the LCD.
 *
 *  returns: Nothing.
 */
void lcdData(char data)
{
	disableInt(); 					//Disable the interrupts, to avoid corruption of the sent data

	GPIO_PORTA_DATA_R &=~(0x0C);	//Clear RS and EN
	GPIO_PORTA_DATA_R |= (1<<2);	//RS HIGH
	GPIO_PORTB_DATA_R =(data>>4);	//Write the upper nibble of the data
	GPIO_PORTA_DATA_R |=(1<<3);		//EN HIGH

	delayMicroseconds(200);
	GPIO_PORTA_DATA_R &=~(0x0C); 	//Clear RS and EN
	GPIO_PORTA_DATA_R |= (1<<2); 	//RS HIGH
	GPIO_PORTB_DATA_R = data;  		//Write the lower nibbleof the data
	GPIO_PORTA_DATA_R |=(1<<3); 	//EN HIGH

	delayMicroseconds(200);
	GPIO_PORTA_DATA_R &=~(0x0C); 	//Clear RS and EN
	delayMicroseconds(400);

	enableInt(); 					//Enable the interrupts
}



/*
 * Function:  lcdCmd
 * --------------------
 * Sends command (byte) to the LCD module in 4bit mode (nibble by nibble).
 *
 *  cmd: Command to send to the LCD.
 *
 *  returns: Nothing.
 */
void lcdCmd(char cmd)
{
	disableInt();					//Disable the interrupts, to avoid corruption of the sent data
	GPIO_PORTA_DATA_R &=~(0x0C);	//Clear RS and EN
	GPIO_PORTB_DATA_R =(cmd>>4);	//Send the upper nibble of the command
	GPIO_PORTA_DATA_R |=(1<<3);		//EN HIGH

	delay(1);
	GPIO_PORTA_DATA_R &=~(0x0C);	//Clear RS and EN
	GPIO_PORTB_DATA_R =(cmd);		//Send the lower nibble of the command
	GPIO_PORTA_DATA_R |=(1<<3);		//EN HIGH

	delay(1);
	GPIO_PORTA_DATA_R &=~(0x0C);	//Clear RS and EN
	delay(2);

	enableInt(); 					//Enable the interrupts
}



/*
 * Function:  lcdPrint
 * --------------------
 * Prints the given string on the LCD.
 *
 *  str: Pointer to the location of the string.
 *  line: Determines the line position on the lCD. 0: Upper line, 1: Lower line. 
 *
 *  returns: Nothing.
 */
void lcdPrint(char *str , int line)
{
	if(!line){
		lcdCmd(0x02); 		//Places the cursor on the upper line
	}else{
		lcdCmd(0xC0); 		//Places the cursor on the lower line
	}
	int i = 0;
	while(str[i] != '\0'){	//Iterates over the string, till a null character appears
		lcdData(str[i]); 	//Writes the character on the lcd
		i++;
	}
}



/*
 * Function:  lcdInit
 * --------------------
 * Initializes the microcontroller and the LCD.
 *
 *
 *  returns: Nothing.
 */
void lcdInit(void)
{
	/*
		Initializing Ports B and A
	*/
	SYSCTL_RCGCGPIO_R |= 0x03; 				//Enable the clock for GPIO ports A and B (bits 0 and 1)
	while ((SYSCTL_PRGPIO_R&0x03)==0){};	//Wait for GPIO clock to start
	GPIO_PORTB_DIR_R |= 0x0F;				//Set the direction for PB0-3 as output
	GPIO_PORTB_DEN_R |= 0x0F;				//Enables the digital I/O for PB0-3
	GPIO_PORTA_DIR_R |= 0x0C;				//Set the direction for PA2,3 as output
	GPIO_PORTA_DEN_R |= 0x0C;				//Enables the digital I/O for PA2,3


	/*
		Initializing the LCD
	*/
	GPIO_PORTA_DATA_R &=~(0x0C); 			//Clear RS and EN
	GPIO_PORTB_DATA_R = 0x00; 				//Send the upper nibble for the 4bit mode command (0010)
	GPIO_PORTA_DATA_R |=(1<<3); 			//EN HIGH
	delay(5);
	GPIO_PORTA_DATA_R &=~(0x0C); 			//Clear RS and EN

	delay(20);
	GPIO_PORTA_DATA_R &=~(0x0C); 			//Clear RS and EN
	GPIO_PORTB_DATA_R = 0x2; 				//Send the lower nibble for the 4bit mode command (0010)
	GPIO_PORTA_DATA_R |=(1<<3); 			//EN HIGH
	delay(5);
	GPIO_PORTA_DATA_R &=~(0x0C); 			//Clear RS and EN

	lcdCmd(0x28);							//2 line, 5*7 matrix in 4-bit mode
	lcdClear();
	lcdHome();
	lcdOn();
}



/*
 * Function:  swInit
 * --------------------
 * Initializes the microcontroller for the switches 1 and 2 (PF0,4).
 *
 *
 *  returns: Nothing.
 */
void swInit(void)
{
	SYSCTL_RCGCGPIO_R |= 0x20;				//Enable the clock for GPIO port F (bits 5)
	while ((SYSCTL_PRGPIO_R&0x20)==0){};	//Wait for GPIO clock to start
	GPIO_PORTF_LOCK_R = 0x4C4F434B;			//Unlock PF
	GPIO_PORTF_CR_R |=0x11;					//Allow changes to PF
	GPIO_PORTF_DIR_R &= ~0x11;				//Set the direction for PF0,4 as input
	GPIO_PORTF_DEN_R |= 0x11;				//Enables the digital I/O for PF0,4
	GPIO_PORTF_PUR_R |= 0x11;				//Enbale enable internal pull up for PF0,4
}



/*
 * Function:  redLedInit
 * --------------------
 * Initializes the microcontroller for the red LED PF1.
 *
 *
 *  returns: Nothing.
 */
void redLedInit(void)
{
	SYSCTL_RCGCGPIO_R |= 0x20;					//Enable the clock for GPIO port F (bit 5)
	while ((SYSCTL_PRGPIO_R&0x20)==0){};		//Wait for GPIO clock to start
	GPIO_PORTF_DIR_R |= 0x02;					//Set the direction for PF1 as output
	GPIO_PORTF_DEN_R |= 0x02;					//Enables the digital I/O for PF1
}



/*
 * Function:  uart2Init
 * --------------------
 * Initializes the microcontroller UART module 2.
 *
 *
 *  returns: Nothing.
 */
void uart2Init(void)
{
	SYSCTL_RCGCUART_R |= 0x04;			//Enable clock for UART2
	SYSCTL_RCGCGPIO_R |= 0x8;			//Enable the clock for GPIO port D

	UART2_CTL_R = 0;
										//Set the UART Baud-rate. BRD (Baud-rate devisor) = Clk / (16*Baud_rate), For 9600, BRD = 80M / (16*9600) = 520.833333
	UART2_IBRD_R = 520;					//UART Baud-rate integer devisor. IBRD = integer(BRD) = 520
	UART2_FBRD_R = 53;					//UART Baud-rate fractional devisor. FBRD = integer(fraction_part *64 +0.5) = integer(0.833333 *64 +0.5) = 53
	UART2_CC_R = 0; 					//Set the UART clock source to be as the system clock (80Mhz)
	UART2_LCRH_R = 0x60;				//Sets the word length to be 8bits and disables the parity
	UART2_CTL_R = 0x201;				//Enable UART2 and enable recieving

	GPIO_PORTD_DEN_R = 0xC0;			//Enable the digital I/O for PD6,7
	GPIO_PORTD_AFSEL_R = 0xC0;			//Enable the alternate function of PD6,7
	GPIO_PORTD_AMSEL_R = 0;				//Disable the analog mode for PD6,7
	GPIO_PORTD_PCTL_R = 0x11000000;		//Select the UART peripheral function for PD6,7
	delay(10);
}



/*
 * Function:  uart2Rcv
 * --------------------
 * Recieves a character from UART2.
 *
 *
 *  returns: The recieved character or null ('\0')if no characters available after 8000 tries.
 */
char uart2Rcv(void)
{
	int tries = 0;
	while ((UART2_FR_R & (1 << 4)) != 0){	//Wait for the Rx FIFO to be full
		if(tries>8000){						//Count the tries till 8000
			return '\0';					//If no characters recieved return null. Used to not keep the proccessor waiting for characters.
		}
		tries++;
	} 
	return (unsigned char) UART2_DR_R;		//Return the character recieved in the data register
}



/*
 * Function:  parseSentence
 * --------------------
 * Parses(searches for) a NMEA sentence from the GPS data.
 *
 *  prefix: Pointer to the location of the sentece prefix string.
 *  data: Pointer to the location of the GPS data string.
 *  sen: The parsed sentence.
 *
 *  returns: By referrence the parsed sentence.
 */
void parseSentence(char *prefix ,char *data, char sen[75])
{
	int ctr = 0;							//Prefix iterator
	int i = 0;								//Data iterator
	int found = 0;							//Prefix found flag
	int j = 0;								//Output sentence iterator
	while(data[i] != '\0'){ 				//Iterate over the characters in the GPS data string till a null ('\0') character (end of the string)
		if(found){							//Check if the prefix is found. If found, fill the sentence (sen) with the data till the newline character ('\n')
			while(j<75&&data[i]!= '\n'){
				sen[j] = data[i];
				i++;
				j++;
			}
			while(j<75){					//Fill the rest of the sentence with null ('\0')
				sen[j] = 0;
				j++;
			}
			return;
		}else{								//If not found, keep searching.
			if(data[i] == prefix[ctr]){		//If the current data character equals the next prefix character
				ctr++;
				if(prefix[ctr] == '\0'){	//Keep looking until the end of the prefix
					found = 1;
				}
			}else{							//If the characters mismatch, reset the prefix iterator (ctr)
				ctr = 0;
			}
		}
		i++;
	}
}



/*
 * Function:  getSpeed
 * --------------------
 * Gets the ground speed data from the GPS NMEA sentence "GPVTG". Sentence example:
 *  $GPVTG,172.21,T,,M,1.736,N,3.216,K,A*3F
 * 	The speed required to be extracted is "3.216", which is always located between the 7th and the 8th commas.
 *
 *  sen: Pointer to the location of the sentece string.
 *  speed: The speed value string.
 *
 *  returns: By referrence the speed value as a string.
 */
void getSpeed(char *sen,char speed[10])
{
	int i = 0;						//Sentence iterator
	int ctr = 0;					//Commas counter
	int j = 0;						//Speed data string iterator
	while(sen[i] != '\0'){			//Iterate over characters of the GPS sentence string
		if(sen[i] == ','){			//If the character is a comma ',' increment the comma counter (ctr)
			ctr++;
		}else if(ctr == 7&&j<10){	//If the character is not a comma and the number of previous commas is 7, parse the speed data.
			speed[j] = sen[i];
			j++;
		}
		i++;
	}
	while(j<10){					//Fill the rest of the speed string with null characters
		speed[j] = 0;
		j++;
	}
}



/*
 * Function:  getTime
 * --------------------
 * Gets the time data from the GPS NMEA sentence "GPGLL". Sentence example:
 *  $GPGLL,4711.48920,N,05250.23190,W,161858.00,A,A*77
 * 	The time required to be extracted is "161858.00", which is always located between the 5th and the 7th commas.
 *
 *  sen: Pointer to the location of the sentece string.
 *  time: The time value string.
 *
 *  returns: By referrence the time value as a string.
 */
void getTime(char *sen,char time[6])
{
	int i = 0;						//Sentence iterator
	int ctr = 0;					//Commas counter
	int j = 0;						//Time data string iterator
	while(sen[i] != '\0'){			//Iterate over characters of the GPS sentence string
		if(sen[i] == ','){			//If the character is a comma ',' increment the comma counter (ctr)
			ctr++;
		}else if(ctr == 5&&j<6){	//If the character is not a comma and the number of previous commas is 5, parse the time data.
			time[j] = sen[i];
			j++;
		}
		i++;
	}
	while(j<6){						//Fill the rest of the speed string with null characters
		time[j] = 0;
		j++;
	}
}


void switchMode(){
      switch(mode){
        case 0:
            lcdClearLine(0);
            lcdPrint(distStr,0);
            lcdClearLine(1);
        break;
        case 1:
            lcdClearLine(0);
            lcdPrint(dispStr,0);
            lcdClearLine(1);
        break;
        case 2:
            lcdClearLine(0);
            lcdPrint(coordStr,0);
            lcdClearLine(1);
            delay(500);
        break;
        case 3:
            lcdClearLine(0);
            lcdPrint(speedStr,0);
            lcdClearLine(1);
        break;
        case 4:
            lcdClearLine(0);
            lcdPrint(timeStr,0);
            lcdClearLine(1);
        break;
        default:
        mode = 0;
        break;
        }
}

void nextMode(){
  disableInt();
    if(mode<4){
        mode++;
    }else{
        mode =0;
    }
    switchMode();
    delay(500);
   enableInt();
}

void distanceMode(int printEn){
  if((mode != 0)&&printEn){switchMode();return;}

    if(printEn){
      char dist[15]={0};
      itoa(distSum,dist,10);
      lcdClearLine(0);
      lcdPrint(distStr,0);
      lcdClearLine(1);
      lcdPrint(dist,1);
      lcdData('m');
    }
    double new_coord[2]={0};
    char sen[75]={0};
    parseSentence("GPGLL",gpsData,sen);
      if(sen[1] == ','||sen[1] == '\0'){
        if(printEn){
        lcdClearLine(0);
        lcdPrint(distNoSig,0);
        }
      return;
      }
    char lat[11]={0};
    char lon[12]={0};
    for(int i=0;i<11;i++){
        if(i==10){
            lon[i] = sen[i+14];
        }else{
            lat[i] = sen[i+1];
            lon[i] = sen[i+14];
        }
    }
        //Calculate the average location to minimize errors
        if((lastCoord[0]+lastCoord[1])==0){
        convertCoords(lat,lon,lastCoord);
        }else{
            convertCoords(lat,lon,new_coord);
            if(setFlg){
            new_coord[0] = latAvg;
            new_coord[1] = lonAvg;
            distSum += distance(lastCoord,new_coord);
            lastCoord[0] = new_coord[0];
            lastCoord[1] = new_coord[1];
            setFlg =0;
            latAvg =0;
            lonAvg =0;
            }else{
                if(setCtr == setSize){
                setFlg =1;
                setCtr =0;
                }else{
                latAvg += new_coord[0]/setSize;
                lonAvg += new_coord[1]/setSize;
                setCtr++;
                }
                }

            }
}
void altFunc(){
  disableInt();
       switch(mode){
        case 0:
          distSum = 0 ;
        break;
        case 1:
          originCoord[0] = currentCoord[0];
          originCoord[1] = currentCoord[1];
        break;
        case 2:
        break;
        case 3:
        break;
        case 4:
        break;
        default:
        mode = 0;
        break;
        }
  delay(200);
  enableInt();
}

void displacementMode(){
  int disp = 0;
    char sen[75]={0};
    parseSentence("GPGLL",gpsData,sen);
      if(sen[1] == ','||sen[1] == '\0'){
        lcdClearLine(0);
        lcdPrint(dispNoSig,0);
      return;
      }
    char lat[11]={0};
    char lon[12]={0};
    for(int i=0;i<11;i++){
        if(i==10){
            lon[i] = sen[i+14];
        }else{
            lat[i] = sen[i+1];
            lon[i] = sen[i+14];
        }
    }
      convertCoords(lat,lon,currentCoord);
      if((originCoord[0]+originCoord[1])!=0){
        disp = distance(originCoord,currentCoord);
      }
    char disps[15]={0};
    itoa(disp,disps,10);
    lcdClearLine(0);
    lcdPrint(dispStr,0);
    lcdClearLine(1);
    lcdPrint(disps,1);
    lcdData('m');
    distanceMode(0);

}

void coordsMode(){
  if(mode != 2){switchMode();return;}
  char sen[75]={0};
  char lat[13]={0};
  char lon[13]={0};
  parseSentence("GPGLL",gpsData,sen);
  if(sen[1] == ','||sen[1] == '\0'){
  lcdClearLine(0);
  lcdPrint(coordStr,0);
   lcdClearLine(1);
  lcdPrint(noGps,1);
  return;
  }
  for(int i=0;i<12;i++){
    if(i==11){
    lon[i] = sen[i+14+1];
    }else if(i==10){
      lat[i] = sen[i+1+1];
      lon[i] = sen[i+14];
    }else{
    lat[i] = sen[i+1];
    lon[i] = sen[i+14];
    }
  }
    lcdClearLine(0);
  lcdPrint((char *)lat,0);
      lcdClearLine(1);
  lcdPrint((char *)lon,1);
  distanceMode(0);
}

void speedMode(){
  if(mode != 3){switchMode();return;}
  char sen[75]={0};
  parseSentence("GPVTG",gpsData,sen);
  char val[10]= {0};
  getSpeed(sen,val);
  if(val[0]=='\0'){
       lcdClearLine(0);
  lcdPrint(speedNoSig,0);
  return;
  }
  lcdClearLine(0);
  lcdPrint(speedStr,0);
  lcdClearLine(1);
  lcdPrint((char *)val,1);
  char unit[] = "Km/h";
  int i=0;
  while(unit[i]!='\0'&&i<1024){
    lcdData(unit[i]);
    i++;
  }
  distanceMode(0);
}

void timeMode(){
  if(mode != 4){switchMode();return;}
  char sen[75]={0};
  parseSentence("GPGLL",gpsData,sen);
  char val[6]= {0};
  getTime(sen,val);
  if(val[0]=='\0'){
    lcdClearLine(0);
    lcdPrint(timeNoSig,0);
    return;
  }
  lcdClearLine(0);
  lcdPrint(timeStr,0);
  lcdClearLine(1);
  // lcdPrint((char *)val,1);
  lcdCmd(0xC0);
  int j = 0;
  while(val[j] != '\0'&&j<8){
    if(j%2 == 0&&j!=0){
      lcdData(':');
    }
    lcdData(val[j]);
    j++;
  }
  char unit[] = " UTC";
  int i=0;
  while(unit[i]!='\0'&&i<1024){
    lcdData(unit[i]);
    i++;
  }
  distanceMode(0);
}

void disableInt(){
  detachInterrupt(PF_4);
  detachInterrupt(PF_0);
}

void enableInt(){
  attachInterrupt(PF_4, nextMode, FALLING);
  attachInterrupt(PF_0, altFunc, FALLING);
}


void setup(){
  delay(200);
  Serial.begin(9600);
  swInit();
  redLedInit();
  uart2Init();
  delay(20);
  lcdInit();
  delay(500);
  lcdClear();
  lcdHome();

    GPIO_PORTF_DATA_R |=0x02;
     delay(200);
     GPIO_PORTF_DATA_R &=~(0x02);
     delay(200);
        GPIO_PORTF_DATA_R |=0x02;
     delay(200);
     GPIO_PORTF_DATA_R &=~(0x02);
    enableInt();
    switchMode();
    lcdClearLine(1);
    lcdPrint(noGps,1);
}

void loop(){
    // unsigned char sw2 = !((GPIO_PORTF_DATA_R & 0x01)>>0);
    // unsigned char sw1 = !((GPIO_PORTF_DATA_R & 0x10)>>4);

    char tmp = uart2Rcv();
    if(tmp != '\0'){
      // memset(c, 0, sizeof c);
    int i=0;
    while((tmp!='\0')&&i<512&&!dataReady){
        gpsData[i] = tmp; 
        tmp = uart2Rcv();
        i++;
        }
    while(i<512){
      gpsData[i] = 0;
      i++;
    }
        dataReady = 1;
        // Serial.println(c);
        return;
    }
    if(dataReady){
        dataReady = 0;
        switch(mode){
        case 0:
          distanceMode(1);
        break;
        case 1:
          displacementMode();
        break;
        case 2:
          coordsMode();
        break;
        case 3:
          speedMode();
        break;
        case 4:
          timeMode();
        break;
        default:
        mode = 0;
        break;
        }
    }
  }

