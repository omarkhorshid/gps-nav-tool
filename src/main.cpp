#include "math.h"
#include "register_definition.h"
#include "Energia.h"

void disableInt();
void enableInt();

char c[512];
bool dataReady = 0;
int dist_sum = 0;
double last_coord[2] = {0};
double lat_avg = 0;
double lon_avg = 0;
int set_flg = 0;
int set_ctr = 0;
int set_size =3;
unsigned int mode = 0; //0: Distance, 1: Displacement, 2: Coords, 3: Speed, 4: Atomic Clock
unsigned int lst_btn_state = 0 ;
double current_coord[2] = {0};
double origin_coord[2] = {0};

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


int distance(double p1[2], double p2[2]){
    float pi = 3.141592654;
    float p = pi/180;
    float a = 0.5 - cos((p2[0]-p1[0])*p)/2 + cos(p1[0]*p) * cos(p2[0]*p) * (1-cos((p2[1]-p1[1])*p))/2;
    return (int)(round((12742 * asin(sqrt(a)))*1000));
    
}
void convertCoords(char lat[10],char lon[10],double pos[2]){
    double deg_lat = 0;
    double deg_lon = 0;
    double dec_lat = 0;
    double dec_lon = 0;
    double latd = strtod(lat,NULL);
    double lond = strtod(lon,NULL);
    deg_lat = latd/100;
    deg_lon = lond/100;
    dec_lat = latd - ((int)deg_lat)*100;
    dec_lon = lond - ((int)deg_lon)*100;
    
    pos[0] = (int)deg_lat+(dec_lat/60);
    pos[1] = (int)deg_lon+(dec_lon/60);
}



void lcdData(char a){
  disableInt();
  GPIO_PORTA_DATA_R &=~(0x0C); //Clear RS and EN
  GPIO_PORTA_DATA_R |= (1<<2); //RS HIGH
  GPIO_PORTB_DATA_R =(a>>4);   //Write the upper nibble
  GPIO_PORTA_DATA_R |=(1<<3); //EN HIGH
  delayMicroseconds(200);
  GPIO_PORTA_DATA_R &=~(0x0C); //Clear RS and EN
  GPIO_PORTA_DATA_R |= (1<<2); //RS HIGH
  GPIO_PORTB_DATA_R = a;  //Write the lower nibble
  GPIO_PORTA_DATA_R |=(1<<3); //EN HIGH
  delayMicroseconds(200);
  GPIO_PORTA_DATA_R &=~(0x0C); //Clear RS and EN
  delayMicroseconds(400);
  enableInt();
}

void lcdCmd(char i){
  disableInt();
  GPIO_PORTA_DATA_R &=~(0x0C); //Clear RS and EN
  GPIO_PORTB_DATA_R =(i>>4);
  GPIO_PORTA_DATA_R |=(1<<3); //EN HIGH
  delay(1);
  GPIO_PORTA_DATA_R &=~(0x0C); //Clear RS and EN
  GPIO_PORTB_DATA_R =(i);
  GPIO_PORTA_DATA_R |=(1<<3); //EN HIGH
  delay(1);
  GPIO_PORTA_DATA_R &=~(0x0C); //Clear RS and EN
  delay(2);
  enableInt();
}

void lcdClear(){
  lcdCmd(0x01);
}

void lcdHome(){
  lcdCmd(0x02);
}

void lcdOn(){
  lcdCmd(0x0C);
}

void lcdPrint(char *str , int line){
  if(!line){
      lcdCmd(0x02);
  }else{
      lcdCmd(0xC0);
  }
    int i = 0;
    while(str[i] != '\0'){
      lcdData(str[i]);
      i++;
    }
}

void lcdClearLine(int line){
  lcdPrint((char *)"                ",line);
}

void lcdInit(){
 //Enable Ports B and A (2&3)
  SYSCTL_RCGCGPIO_R |= 0x03; 
  while ((SYSCTL_PRGPIO_R&0x03)==0){};
  GPIO_PORTB_DIR_R |= 0x0F;
  GPIO_PORTB_DEN_R |= 0x0F;
  GPIO_PORTA_DIR_R |= 0x0C;
  GPIO_PORTA_DEN_R |= 0x0C;


  GPIO_PORTA_DATA_R &=~(0x0C); //Clear RS and EN
  GPIO_PORTB_DATA_R = 0x00; //0010 4Bit mode
  GPIO_PORTA_DATA_R |=(1<<3); //EN HIGH
  delay(5);
  GPIO_PORTA_DATA_R &=~(0x0C); //Clear RS and EN

  delay(20);
  //set 4 bit mode
  GPIO_PORTA_DATA_R &=~(0x0C); //Clear RS and EN
  GPIO_PORTB_DATA_R = 0x2; //0010 4Bit mode
  GPIO_PORTA_DATA_R |=(1<<3); //EN HIGH
  delay(5);
  GPIO_PORTA_DATA_R &=~(0x0C); //Clear RS and EN

  lcdCmd(0x28);
  lcdClear();
  lcdHome();
  lcdOn();
}


void swInit(void){ // the initialization of SW1 and SW2 from port f
  SYSCTL_RCGCGPIO_R |= 0x20;
  while ((SYSCTL_PRGPIO_R&0x20)==0){};
  GPIO_PORTF_LOCK_R = 0x4C4F434B;
  GPIO_PORTF_CR_R |=0x11;
  GPIO_PORTF_DIR_R &= ~0x11;
  GPIO_PORTF_DEN_R |= 0x11;
  GPIO_PORTF_PUR_R |= 0x11;
}
void redLedInit(void){ // the initialization of red led from port f
  SYSCTL_RCGCGPIO_R |= 0x20;
  while ((SYSCTL_PRGPIO_R&0x20)==0){};
  GPIO_PORTF_DIR_R |= 0x02;
  GPIO_PORTF_DEN_R |= 0x02;

}

void uart2Init(void){ // the initialization of RX and TX pins in tiva to have the data from the module which will be in D6 and D7
  SYSCTL_RCGCUART_R |= 0x04;
  SYSCTL_RCGCGPIO_R |= 0x8;

  UART2_CTL_R = 0;
  UART2_IBRD_R = 520;
  UART2_FBRD_R = 53;
  UART2_CC_R = 0;
  UART2_LCRH_R = 0x60;
  UART2_CTL_R = 0x201;

  GPIO_PORTD_DEN_R = 0xC0;
  GPIO_PORTD_AFSEL_R = 0xC0;
  GPIO_PORTD_AMSEL_R = 0;
  GPIO_PORTD_PCTL_R = 0x11000000;
  delay(10);
}
char uart2Rcv(void)  
{
  char data;
  int tries = 0;
  while ((UART2_FR_R & (1 << 4)) != 0){
    if(tries>8000){
      return '\0';
    }
    tries++;
  } /* wait until Rx buffer is not full */
  data = UART2_DR_R; /* before giving it another byte */
  return (unsigned char) data;
}

void parseSentence(char *prefix ,char *data, char sen[75]){
    int ctr = 0;
    int i = 0;
    int found = 0;
    int j = 0;
    while(data[i] != '\0'){
        if(found){
            while(j<75&&data[i]!= '\n'){
                sen[j] = data[i];
                i++;
                j++;
            }
            while(j<75){
                sen[j] = 0;
                j++;
            }
            return;
        }else{
        if(data[i] == prefix[ctr]){
            ctr++;
            if(prefix[ctr] == '\0'){
                found = 1;
            }
        }else{
            ctr = 0;
        }
        }
        i++;
    }
}

void getSpeed(char *sen,char speed[10]){
    int i = 0;
    int ctr = 0;
    int j = 0;
    while(sen[i] != '\0'){
      if(sen[i] == ','){
        ctr++;
      }else if(ctr == 7&&j<10){
              speed[j] = sen[i];
              j++;
          }
      i++;
    }
    while(j<10){
        speed[j] = 0;
        j++;
    }
}
void getTime(char *sen,char time[6]){
    int i = 0;
    int ctr = 0;
    int j = 0;
    while(sen[i] != '\0'){
      if(sen[i] == ','){
        ctr++;
      }else if(ctr == 5&&j<6){
              time[j] = sen[i];
              j++;
          }
      i++;
    }
    while(j<6){
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
      itoa(dist_sum,dist,10);
      lcdClearLine(0);
      lcdPrint(distStr,0);
      lcdClearLine(1);
      lcdPrint(dist,1);
      lcdData('m');
    }
    double new_coord[2]={0};
    char sen[75]={0};
    parseSentence("GPGLL",c,sen);
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
        if((last_coord[0]+last_coord[1])==0){
        convertCoords(lat,lon,last_coord);
        }else{
            convertCoords(lat,lon,new_coord);
            if(set_flg){
            new_coord[0] = lat_avg;
            new_coord[1] = lon_avg;
            dist_sum += distance(last_coord,new_coord);
            last_coord[0] = new_coord[0];
            last_coord[1] = new_coord[1];
            set_flg =0;
            lat_avg =0;
            lon_avg =0;
            }else{
                if(set_ctr == set_size){
                set_flg =1;
                set_ctr =0;
                }else{
                lat_avg += new_coord[0]/set_size;
                lon_avg += new_coord[1]/set_size;
                set_ctr++;
                }
                }

            }
}
void altFunc(){
  disableInt();
       switch(mode){
        case 0:
          dist_sum = 0 ;
        break;
        case 1:
          origin_coord[0] = current_coord[0];
          origin_coord[1] = current_coord[1];
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
    parseSentence("GPGLL",c,sen);
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
      convertCoords(lat,lon,current_coord);
      if((origin_coord[0]+origin_coord[1])!=0){
        disp = distance(origin_coord,current_coord);
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
  parseSentence("GPGLL",c,sen);
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
  parseSentence("GPVTG",c,sen);
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
  parseSentence("GPGLL",c,sen);
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
        c[i] = tmp; 
        tmp = uart2Rcv();
        i++;
        }
    while(i<512){
      c[i] = 0;
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
