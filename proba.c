/*******************************************************
This program was created by the CodeWizardAVR V4.02
Automatic Program Generator
© Copyright 1998-2024 Pavel Haiduc, HP InfoTech S.R.L.
http://www.hpinfotech.ro

Project :
Version :
Date    :  5/ 5/2024
Author  :
Company :
Comments:


Chip type               : ATmega164
Program type            : Application
AVR Core Clock frequency: 10.000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 256
*******************************************************/

// I/O Registers definitions
#include <mega164.h>

char in; //intrare
char out;  //iesire
char Q; //stare CLS
char eroare; //eroare=0 eroare introducere pin
char S; //stare PS
char Q1;//copie a lui Q

char *TAB1[81]; //tabela de adrese

//tabele de semnale relevante CLS
//pin 1            
char A0[] = {5, 1, 16 , 0}; 
char A1[] = {4, 2 , 16 , 1};
char A2[] = {2, 3 , 16 , 2};
char A3[] = {7, 4 , 16 , 3};
//pin 2
char A4[] = {2, 5 , 16 , 4};   
char A5[] = {7, 6, 16 , 5};   
char A6[] = {4, 7 , 16 , 6};
char A7[] = {0, 8 , 16 , 7};
//pin 3
char A8[] = {7, 9 , 16 , 8};   
char A9[] = {1, 10 , 16 , 9};
char A10[] = {0, 11, 16 , 10};   
char A11[] = {1, 12, 16 , 11};
//pin 4
char A12[] = {4, 13, 16 , 12};   
char A13[] = {2, 14, 16 , 13};
char A14[] = {1, 15, 16 , 14};
char A15[] = {5, 16, 16 , 15};   
//pin 5
char A16[] = {8, 17 , 16 , 16};  
char A17[] = {2, 18 , 16 , 17};
char A18[] = {2, 19 , 16 , 18};
char A19[] = {6, 20 , 16 , 19};
//pin 6
char A20[] = {0, 21, 16 , 20};     
char A21[] = {4, 22 , 16 , 21};
char A22[] = {2, 23 , 16 , 22};
char A23[] = {0, 24 , 16 , 23};
//pin 7
char A24[] = {1, 25 , 16 , 24};     
char A25[] = {1, 26 , 16 , 25};
char A26[] = {8, 27 , 16 , 26};
char A27[] = {4, 28 , 16 , 27};
//pin 8
char A28[] = {0, 29 , 16 , 28};    
char A29[] = {7, 30 , 16 , 29};
char A30[] = {7, 31 , 16 , 30};
char A31[] = {9, 32 , 16 , 31};
 //pin 9
char A32[] = {2, 33 , 16 , 32};   
char A33[] = {0, 34 , 16 , 33};
char A34[] = {8, 35 , 16 , 34};
char A35[] = {3, 36 , 16 , 35};
//pin 10
char A36[] = {1, 37 , 16 , 36};   
char A37[] = {7, 38 , 16 , 37};
char A38[] = {8, 39 , 16 , 38};
char A39[] = {4, 40 , 16 , 39};
//pin 11
char A40[] = {6, 41 , 16 , 40};   
char A41[] = {4, 42 , 16 , 41};
char A42[] = {2, 43 , 16 , 42};
char A43[] = {5, 44 , 16 , 43};
//pin 12
char A44[] = {5, 45 , 16 , 44};     
char A45[] = {3, 46 , 16 , 45};
char A46[] = {0, 47 , 16 , 46};
char A47[] = {1, 48 , 16 , 47};
 //pin 13
char A48[] = {0, 49 , 16 , 48};    
char A49[] = {6, 50 , 16 , 49};
char A50[] = {3, 51 , 16 , 50};
char A51[] = {7, 52 , 16 , 51};
//pin 14
char A52[] = {9, 53 , 16 , 52};    
char A53[] = {1, 54 , 16 , 53};
char A54[] = {0, 55 , 16 , 54};
char A55[] = {7, 56 , 16 , 55};
//pin 15
char A56[] = {5, 57 , 16 , 56};     
char A57[] = {7, 58 , 16 , 57};
char A58[] = {6, 59 , 16 , 58};
char A59[] = {6, 60 , 16 , 59};
//pin 16
char A60[] = {3, 61 , 16 , 60};    
char A61[] = {9, 62 , 16 , 61};
char A62[] = {1, 63 , 16 , 62};
char A63[] = {1, 64 , 16 , 63};
//pin 17
char A64[] = {0, 65 , 16 , 64};     
char A65[] = {0, 66 , 16 , 65};
char A66[] = {1, 67 , 16 , 66};
char A67[] = {2, 68 , 16 , 67};
//pin 18
char A68[] = {3, 69 , 16 , 68};       
char A69[] = {3, 70 , 16 , 69};
char A70[] = {2, 71 , 16 , 70};
char A71[] = {9, 72 , 16 , 71};
 //pin 19
char A72[] = {0, 73 , 16 , 72};      
char A73[] = {2, 74 , 16 , 73};
char A74[] = {2, 75 , 16 , 74};
char A75[] = {7, 76 , 16 , 75};
//pin 20
char A76[] = {3, 77 , 16 , 76};       
char A77[] = {9, 78 , 16 , 77};
char A78[] = {2, 79 , 16 , 78};
char A79[] = {0, 80 , 16 , 79};
char A80[]={16,80};  //terminator

void card_number_goes_to_pin(void)   //am ales valori ale nr_card=[0A,1E] deoarece cifra 0x00(nr_card) nu era luata in calcul 
{                                   //si la introducerea unei cifre {1,2,3,4..,9} trecea in alta stare CLS 
    switch(PIND)   //switch pentru a afla pinul corespunzator fiecarui cont , in functie de nr_card introdus se trece la starea Q corespunzatoare
            {
            case 0x01: Q=0;
            break;
            case 0x02: Q=4;
            break;
            case 0x03: Q=8;
            break;
            case 0x04: Q=12;
            break;
            case 0x09: Q=16;
             break;   
            case 0x0A: Q=20;
            break;
            case 0x0B: Q=24;
            break;
            case 0x0C: Q=28;
            break;
            case 0x0D: Q=32;
            break;
            case 0x0E: Q=36;
            break;
            case 0x11: Q=40;
            break;
            case 0x12: Q=44;
            break;
            case 0x13: Q=48;
            break;
            case 0x39: Q=52;
            break;
            case 0x3A: Q=56;
            break;
            case 0x3B: Q=60;
            break;
            case 0x3C: Q=64;
            break;
            case 0x3D: Q=68;
            break;
            case 0x3E: Q=72;
            break;
            case 0x3F: Q=76;
            break;
            }
}
void PIN_incorect(void)
{
    char x = 0xFF;  //intial led oprit
    char cnt =0;
    while (cnt <30)  //30 intreruperi = 3s -->1s = 10 intreruperi
    {
    
        if ( in == 0x40 )   // daca SW6 pentru incheierea pinului este aprins se executa
        {
            switch(x)
            {
            case 0xFF: x=0xF7; PORTB = x;   //LED3 aprins
            break;            
            case 0xF7: x=0xFF; PORTB = x;  //LED3 stins
            break;
            }       
        }
        cnt ++;
    }         
}

//tabela de adevar CLC
char TAB[8] = {0xFF, 0xFE, 0xFC, 0xFF, 0xFF, 0xFA, 0xF8, 0xFF};  //SOLD,RETRAGERE,CHITANTA sunt active in 0


//fct CLC
void clc(void)
{
	char tmp;
	tmp = in & 0x07; //masca 0x07, SOLD-SW0,RETRAGERE-SW1,CHITANTA-SW2
	out = TAB[tmp];  //actiunile sunt reprezentate prin LED-urile 0,1,2
    PORTB = out;
}

    
//fct CLS
void cls(void)
{
	char i;
	char *adr;
	char ready;

	adr = TAB1[Q];
	i = 0;
	ready = 0;
	while (!ready)
		{
		if(in == *(adr + i)) {Q = *(adr + i + 1); ready = 1;}
		else if(*(adr + i) == 16) ready = 1;
		else i = i + 2;
		}
}


// Timer 0 overflow interrupt service routine
interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{
	// Reinitialize Timer 0 value
	TCNT0 = 0x3C;

	// Place your code here
	
	//initializari variabile
	
	in = PIND;
	switch (S)
		{
	case 0:
		if(in == 0x80) {S = 1;}  //SW7  introducere pin
    break;
  	case 1:
        eroare=1;    //resetare eroare
        if( Q == 0 )
        {
            card_number_goes_to_pin();
            Q1=Q;
        }
		cls(); //se testeaza pin
		if(in == 0x40) {S = 2;} //SW6   incheiere pin
	break;
	case 2:
		if((Q1+4)==Q)   //comanda Q%4==0 nu era buna deoarece la introducerea gresita a tuturor cifrelor intra in CLC
			{
			PORTB = PORTB | 0x0F;
			clc();
			if(in == 0x08) {S = 3;} //stop clc nu se mai solicita nicio comanda si se trece in urmatoarea stare
			}
		else
			{  
            eroare = 0;   //eroarea devine 0-eroare introducere pin
			PIN_incorect();   //se trece in fct pentru pin incorect
            Q = 0;
			S = 0;      //se reseteaza stare S si starea Q pentru CLS
            PIND = 0x00;   //se alctualizeaza toate SW
			}
	break;
	case 3:
		PORTB = PORTB | 0xFF; //stinge LED-uri 
        PIND = 0x00;   //se actualizeaza toate SW
		S = 0;
        Q = 0;     //se modifica starea S si Q pentru a realua procesul daca se doreste folosirea altui cont
	break;
	PORTB = out;  //scrie iesirea 
	}


}

// Declare your global variables here

void main(void)
{
	// Declare your local variables here
    eroare = 1;
	PORTB = (eroare << 3) | 0xF7;   // LED3 pentru introducere incorecta PIN   
    //initializare adrese tabele de semnale relevante
	TAB1[0] = A0;
	TAB1[1] = A1;
	TAB1[2] = A2;
	TAB1[3] = A3;
	TAB1[4] = A4;
    TAB1[5] = A5;
    TAB1[6] = A6;
    TAB1[7] = A7;
    TAB1[8] = A8;
    TAB1[9] = A9;
    TAB1[10] = A10;
    TAB1[11] = A11;
    TAB1[12] = A12;
    TAB1[13] = A13;
    TAB1[14] = A14;
    TAB1[15] = A15;
    TAB1[16] = A16;
    TAB1[17] = A17;
    TAB1[18] = A18;
    TAB1[19] = A19;
    TAB1[20] = A20;
    TAB1[21] = A21;
    TAB1[22] = A22;
    TAB1[23] = A23;
    TAB1[24] = A24;
    TAB1[25] = A25;
    TAB1[26] = A26; 
    TAB1[27] = A27;
    TAB1[28] = A28; 
    TAB1[29] = A29;
	TAB1[30] = A30;
	TAB1[31] = A31;
	TAB1[32] = A32;
	TAB1[33] = A33;
    TAB1[34] = A34;
    TAB1[35] = A35;
    TAB1[36] = A36;
    TAB1[37] = A37;
    TAB1[38] = A38;
    TAB1[39] = A39;
	TAB1[40] = A40;
	TAB1[41] = A41;
	TAB1[42] = A42;
	TAB1[43] = A43;
    TAB1[44] = A44;
    TAB1[45] = A45;
    TAB1[46] = A46;
    TAB1[47] = A47;
    TAB1[48] = A48;
    TAB1[49] = A49;
	TAB1[50] = A50;
	TAB1[51] = A51;
	TAB1[52] = A52;
	TAB1[53] = A53;
    TAB1[54] = A54;
    TAB1[55] = A55;
    TAB1[56] = A56;
    TAB1[57] = A57;
    TAB1[58] = A58;
    TAB1[59] = A59;
	TAB1[60] = A60;
	TAB1[61] = A61;
	TAB1[62] = A62;
	TAB1[63] = A63;
    TAB1[64] = A64;
    TAB1[65] = A65;
    TAB1[66] = A66;
    TAB1[67] = A67;
    TAB1[68] = A68;
    TAB1[69] = A69;
	TAB1[70] = A70;
	TAB1[70] = A70;
	TAB1[71] = A71;
	TAB1[72] = A72;
    TAB1[73] = A73;
    TAB1[74] = A74;
    TAB1[75] = A75;
    TAB1[76] = A76;
    TAB1[77] = A77;
    TAB1[78] = A78;
    TAB1[79] = A79;
    TAB1[80] = A80;
    

	S = 0;
	Q = 0;
	out = 0xFF;   //initial S=0,Q=0 si LED-uri oprite
	// Clock Oscillator division factor: 1
#pragma optsize-
	CLKPR = (1 << CLKPCE);
	CLKPR = (0 << CLKPCE) | (0 << CLKPS3) | (0 << CLKPS2) | (0 << CLKPS1) | (0 << CLKPS0);
	#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
	#endif

	// Input/Output Ports initialization
	// Port A initialization
	// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
	DDRA = (0 << DDA7) | (0 << DDA6) | (0 << DDA5) | (0 << DDA4) | (0 << DDA3) | (0 << DDA2) | (0 << DDA1) | (0 << DDA0);
	// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
	PORTA = (0 << PORTA7) | (0 << PORTA6) | (0 << PORTA5) | (0 << PORTA4) | (0 << PORTA3) | (0 << PORTA2) | (0 << PORTA1) | (0 << PORTA0);

	// Port B initialization
	// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out
	DDRB = (1 << DDB7) | (1 << DDB6) | (1 << DDB5) | (1 << DDB4) | (1 << DDB3) | (1 << DDB2) | (1 << DDB1) | (1 << DDB0);
	// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0
	PORTB = (0 << PORTB7) | (0 << PORTB6) | (0 << PORTB5) | (0 << PORTB4) | (0 << PORTB3) | (0 << PORTB2) | (0 << PORTB1) | (0 << PORTB0);

	// Port C initialization
	// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
	DDRC = (0 << DDC7) | (0 << DDC6) | (0 << DDC5) | (0 << DDC4) | (0 << DDC3) | (0 << DDC2) | (0 << DDC1) | (0 << DDC0);
	// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
	PORTC = (0 << PORTC7) | (0 << PORTC6) | (0 << PORTC5) | (0 << PORTC4) | (0 << PORTC3) | (0 << PORTC2) | (0 << PORTC1) | (0 << PORTC0);

	// Port D initialization
	// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
	DDRD = (0 << DDD7) | (0 << DDD6) | (0 << DDD5) | (0 << DDD4) | (0 << DDD3) | (0 << DDD2) | (0 << DDD1) | (0 << DDD0);
	// State: Bit7=P Bit6=P Bit5=P Bit4=P Bit3=P Bit2=P Bit1=P Bit0=P
	PORTD = (1 << PORTD7) | (1 << PORTD6) | (1 << PORTD5) | (1 << PORTD4) | (1 << PORTD3) | (1 << PORTD2) | (1 << PORTD1) | (1 << PORTD0);

	// Timer/Counter 0 initialization
	// Clock source: System Clock
	// Clock value: 9.766 kHz
	// Mode: Normal top=0xFF
	// OC0A output: Disconnected
	// OC0B output: Disconnected
	// Timer Period: 20.07 ms
	TCCR0A = (0 << COM0A1) | (0 << COM0A0) | (0 << COM0B1) | (0 << COM0B0) | (0 << WGM01) | (0 << WGM00);
	TCCR0B = (0 << WGM02) | (1 << CS02) | (0 << CS01) | (1 << CS00);
	TCNT0 = 0x3C;
	OCR0A = 0x00;
	OCR0B = 0x00;

	// Timer/Counter 1 initialization
	// Clock source: System Clock
	// Clock value: Timer1 Stopped
	// Mode: Normal top=0xFFFF
	// OC1A output: Disconnected
	// OC1B output: Disconnected
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer1 Overflow Interrupt: Off
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: Off
	// Compare B Match Interrupt: Off
	TCCR1A = (0 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (0 << WGM11) | (0 << WGM10);
	TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (0 << WGM12) | (0 << CS12) | (0 << CS11) | (0 << CS10);
	TCNT1H = 0x00;
	TCNT1L = 0x00;
	ICR1H = 0x00;
	ICR1L = 0x00;
	OCR1AH = 0x00;
	OCR1AL = 0x00;
	OCR1BH = 0x00;
	OCR1BL = 0x00;

	// Timer/Counter 2 initialization
	// Clock source: System Clock
	// Clock value: Timer2 Stopped
	// Mode: Normal top=0xFF
	// OC2A output: Disconnected
	// OC2B output: Disconnected
	ASSR = (0 << EXCLK) | (0 << AS2);
	TCCR2A = (0 << COM2A1) | (0 << COM2A0) | (0 << COM2B1) | (0 << COM2B0) | (0 << WGM21) | (0 << WGM20);
	TCCR2B = (0 << WGM22) | (0 << CS22) | (0 << CS21) | (0 << CS20);
	TCNT2 = 0x00;
	OCR2A = 0x00;
	OCR2B = 0x00;

	// Timer/Counter 0 Interrupt(s) initialization
	TIMSK0 = (0 << OCIE0B) | (0 << OCIE0A) | (1 << TOIE0);

	// Timer/Counter 1 Interrupt(s) initialization
	TIMSK1 = (0 << ICIE1) | (0 << OCIE1B) | (0 << OCIE1A) | (0 << TOIE1);

	// Timer/Counter 2 Interrupt(s) initialization
	TIMSK2 = (0 << OCIE2B) | (0 << OCIE2A) | (0 << TOIE2);

	// External Interrupt(s) initialization
	// INT0: Off
	// INT1: Off
	// INT2: Off
	// Interrupt on any change on pins PCINT0-7: Off
	// Interrupt on any change on pins PCINT8-15: Off
	// Interrupt on any change on pins PCINT16-23: Off
	// Interrupt on any change on pins PCINT24-31: Off
	EICRA = (0 << ISC21) | (0 << ISC20) | (0 << ISC11) | (0 << ISC10) | (0 << ISC01) | (0 << ISC00);
	EIMSK = (0 << INT2) | (0 << INT1) | (0 << INT0);
	PCICR = (0 << PCIE3) | (0 << PCIE2) | (0 << PCIE1) | (0 << PCIE0);

	// USART0 initialization
	// USART0 disabled
	UCSR0B = (0 << RXCIE0) | (0 << TXCIE0) | (0 << UDRIE0) | (0 << RXEN0) | (0 << TXEN0) | (0 << UCSZ02) | (0 << RXB80) | (0 << TXB80);

	// USART1 initialization
	// USART1 disabled
	UCSR1B = (0 << RXCIE1) | (0 << TXCIE1) | (0 << UDRIE1) | (0 << RXEN1) | (0 << TXEN1) | (0 << UCSZ12) | (0 << RXB81) | (0 << TXB81);

	// Analog Comparator initialization
	// Analog Comparator: Off
	// The Analog Comparator's positive input is
	// connected to the AIN0 pin
	// The Analog Comparator's negative input is
	// connected to the AIN1 pin
	ACSR = (1 << ACD) | (0 << ACBG) | (0 << ACO) | (0 << ACI) | (0 << ACIE) | (0 << ACIC) | (0 << ACIS1) | (0 << ACIS0);
	ADCSRB = (0 << ACME);
	// Digital input buffer on AIN0: On
	// Digital input buffer on AIN1: On
	DIDR1 = (0 << AIN0D) | (0 << AIN1D);

	// ADC initialization
	// ADC disabled
	ADCSRA = (0 << ADEN) | (0 << ADSC) | (0 << ADATE) | (0 << ADIF) | (0 << ADIE) | (0 << ADPS2) | (0 << ADPS1) | (0 << ADPS0);

	// SPI initialization
	// SPI disabled
	SPCR = (0 << SPIE) | (0 << SPE) | (0 << DORD) | (0 << MSTR) | (0 << CPOL) | (0 << CPHA) | (0 << SPR1) | (0 << SPR0);

	// TWI initialization
	// TWI disabled
	TWCR = (0 << TWEA) | (0 << TWSTA) | (0 << TWSTO) | (0 << TWEN) | (0 << TWIE);

	// Globally enable interrupts
#asm("sei")

	while (1)
		{
		// Place your code here

		}
}
