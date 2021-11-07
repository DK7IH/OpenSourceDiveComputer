
//***************************************************************//
//  Dekompressionsprogramm fuer Selbstbau-Tauchcomputer (SBTC3b) //
//  ************************************************************ //
//  Mikrocontroller:  ATMEL AVR ATmega32, 8 MHz                  //
//  Projektname:      SBTC3b                                     //
//  Compiler:         GCC (GNU AVR C-Compiler)                   //
//  Autor:            Peter Rachow - Karlsruhe                   //
//  Letzte Aenderung: 16.07.2011                                 //
//***************************************************************//
//
// Vcc=5V stab. LM2931
//
// Pinbelegung am Mikrocontroller
//
// PIN   Funktion
//
// 1     Taster 1  Taster gegen Masse, interner Pull-up-Widerstand geschaltet.
// 2     Taster 2                             "
// 3     Taster 3                             "
// 6     MOSI
// 7     MISO
// 8     SCK
// 9     REST
// 10    +Vcc
// 11    GND
// 14    RS 232 Rx-Data
// 15    RS 232 Tx-Data
// 18    Display Léitung 11
// 19    Display Léitung 12
// 20    Display Léitung 13
// 21    Display Léitung 14
// 22    Display Léitung 6
// 23    Display Léitung 4
// 24    LED 1 (Rechnung aktiv)
// 25    LED 2 (Dekostufe übertaucht!)
// 28    X-TAL
// 29    X-TAL
// 30    Vcc ADC
// 31    GND
// 32    GND über 0.1uF
// 38    Spannungsteiler 1:5 für Akkuüberwachung
// 39    Temperatursensor KTY 81-210
// 40    Ausgang OP-Amp des Drucksensors p => U 
//          1bar <= p <= 9 bar; 0V <= U <= 2.56 V

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <string.h>


//*****************
//  Benutzermenue  
//*****************
#define MENU_ITEMS 7
char menu_str[MENU_ITEMS][18]={"Luftdruck NN",
                                "Hoehe ueber NN",
                                "Kabinendruck",
                                "Max. ppO2",
                                "Toleranzen",
                        "ppN2 anzeigen",
                        "Einstellungen"};

char menu_unitstr[MENU_ITEMS][6]={"mbar",
                                   "m",
                                   "mbar",
                                   "bar",
                                   "",
                           "",
                           ""};

int menu_digits[MENU_ITEMS] = {-1, -1, -1, 2, 2,  -1, -1}; // Zahl der Ziffern 
int menu_dec[MENU_ITEMS] = {-1, -1, -1, 1, 1, -1, -1};     // Position des Dezimalpunktes 

int show_settings = 0;

//*****************
// EEPROM-Speicher 
//*****************
#include <avr/eeprom.h>
#define MAX_EEPROM_ADR 1023
#define EEPROM_PROF_START 50

void eeprom_store_byte(char);
void clear_flash(char);
void display_profile(void);
void display_rcd(void);
void display_log(void);
int eeprom_byte_count;                 // Positionszeiger fuer EEPROM 

//*******************
// Timer & Interrupt 
//*******************
//#define F_CPU 8000000      // Taktfrequenz im MHz in <util/delay.h>                               
#define INITWAIT 750         // Wartezeit fuer Anzeigewechsel bei Programmstart   
unsigned long runseconds = 0, diveseconds = 0, surf_seconds = 0;

//*********
// M I S C 
//*********
int main(void);
void led(char, char);
int round_depth(int);
int get_keys(void);
void settings(void);

void showtemp(void);
double temp;                            // Aktuelle Temperatur                          
int temp_min;                        // niedrigste Temperatur                        
int temp_maxdepth;                   // Temperatur auf max Tiefe                     

void show_gas(int);
void set_curgas(void);

#define SWITCHDEPTH 10 // Tiefe, bei der von TG- in Oberflaechenmodus 
                       // gewechselt wird [dm].                        
#define SURF_SECONDS_MAX 180 // Oberflaechenzeit, nach der von TG- in OFP-Modus 
                             // gewechselt wird [s].                           

// Softwareversion 
unsigned char softwareversion[3] = {1, 1, 'c'};

void show_accu_voltage(void);
double accu_voltage = 0; //Akkuspannung

//*************
// LCD-Display 
//*************
#define LCD_INST 0x00
#define LCD_DATA 0x01

void lcd_write(char, unsigned char, int);
void set_rs(char);
void set_e(char);
void lcd_init(void);
void lcd_cls(void);
void lcd_linecls(int, int);
void lcd_putchar(int, int, unsigned char);
void lcd_putstring(int, int, char*);
int lcd_putnumber(int, int, int, int, int, char, char);
void wait_ms(int);
void lcd_printdiveinfo(int, int, int);

//*******
// USART 
//*******
#define RX_BUF_SIZE 32

void usart_init(void);
void usart_putc(char);
void clear_rx_buf(void);
char make_crc(int, int);
void sbtc2pc(void);

char rx_buf[RX_BUF_SIZE];
unsigned char rx_buf_cnt = 0;

//***********************
// Dekompressionrechnung 
//***********************
#define NCOMP 16   // Anzahl der Kompartimente des Modells 
#define FN2 0.78   // N2-Anteil im Atemgas                 
#define MAX_DECO_STEPS 10
#define MAXGASES 3

// Gewebekonstanten fuer 16 Kompartimente  
// STICKSTOFF                              
float t05N2[] = {4, 8, 12.5, 18.5, 27, 38.3, 54.3, 77, 109, 146, 187, 239, 305, 390, 498, 635};
float aN2[] = {1.2599, 1, 0.8618, 0.7562, 0.662, 0.5043, 0.441, 0.4,
    0.375, 0.35, 0.3295, 0.3065, 0.2835, 0.261, 0.248, 0.2327};
float bN2[] = {0.505, 0.6514, 0.7222, 0.7825, 0.8126, 0.8434, 0.8693, 0.891,
    0.9092, 0.9222, 0.9319, 0.9403, 0.9477, 0.9544, 0.9602, 0.9653};

// Kompartimentsaettigung 
float piN2[] = {0.72, 0.72, 0.72, 0.72, 0.72, 0.72, 0.72, 0.72,
    0.72, 0.72, 0.72, 0.72, 0.72, 0.72, 0.72, 0.72};

// 3 durch Anwender waehlbare Gasgemische aus O2 und N2 (Gas1 = Luft) 
unsigned char curgas = 0;
double figN2[MAXGASES] = {FN2, 0.36, 0};              // N2-Anteil in 3 Auswahlgasen    

float airp = 0.995;                  // Umgebungsluftdruck in bar am Tauchort        
float airp0 = 0.995;                 // Umgebungsluftdruck in bar auf NN             
float cabinp = 0.75;                 // Kabinendruck im Flugzeug in bar              
int altitude = 0;                    //Hoehe ueber NN                                
int depth = 0, maxdepth = 0;         // Akt. und max. Tiefe [dm]                     
int deepest_decostep = 0;            // Tiefster Dekostopp in dm                     
int deco_minutes_total = 0;          // Gesamtdekozeit in min.                       
char dphase = 0;                     // TG-Phase: 1=tauchen 0=OFP                    
unsigned char f_cons;               // Faktor fuer ab-Modifikation (10facher Wert)  
char show_ppN2 = 0;                  // ppN2 nach TG anzeigen für 16 Kompartimente   

unsigned char rcd_decotime[MAX_DECO_STEPS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// Speicherdaten fuer die Decostufen, die 
// im EEPROM fuer den TG abgelegt werden   

unsigned char rcd_deco_minutes_total = 0;
unsigned char tmp_decotime_total = 0;

unsigned char surfaced = 0, ppo2_exceeded = 0;
unsigned char decostep_skipped = 0, ndt_runout = 0; // Flags fuer Ereignisaufzeichnung im Profilespeicher 
unsigned char temp_low = 0;

float get_water_pressure(int);
void calc_p_inert_gas(int);
float get_water_depth(float);
int calc_ndt(void);
void calc_deco(void);
unsigned int calc_no_fly_time(void);
void get_dsensor(void);
void get_tsensor(void);
void get_vsensor(void);
void set_ab_values(int, unsigned char);
void calc_airp_divesite(char);

//*********************
// ppO2-bezogene Werte 
//*********************
float cns_day = 0, cns_dive;
float otu = 0;
int maxppo2 = 16;  // 1.6 bar 

int calc_ppo2(char);
void calc_cns_otu(void);

//************
// AD-Wandler 
//************
#define ADWAITSTATE 3
int adc_val;
char adc_mode = 0; // 0=Druck, 1=Temperatur, 2=Akkuspannung

//********************************************************//
// Funktionen und Prozeduren fuer Dekompressionsrechnung //
//******************************************************//

// Wasserdruck p.amb aus Tiefe depth  
// berechnen                          
float get_water_pressure(int depth)
{
    return depth * 0.1 + airp;
}

// Inertgaspartialdruck im Gewebe berechnen 
// d: Tiefe in m  Intervall immer 10 sec.   
void calc_p_inert_gas(int d)
{
    unsigned char t1;
    float pamb = get_water_pressure(d) - 0.0627;

    for(t1 = 0; t1 < NCOMP; t1++)
        piN2[t1] += (pamb * figN2[curgas] - piN2[t1]) * (1 - exp((-0.1667 / t05N2[t1]) * log(2)));
}

// Wassertiefe depth aus p.amb berechnen 
float get_water_depth(float pamb)
{
    return (pamb - airp) * 10;
}

// Errechnen der Restnullzeit 
int calc_ndt()
{
    char calcok = 0;      // Flag, ob Rechnung OK ist 
    unsigned char t1;

    int dp = depth * 0.1; // Wassertiefe in m 
    int t0min = 999;

    float te, xN2;
    float piigN2, pamb = get_water_pressure(dp) - 0.0627;

    piigN2 = pamb * figN2[curgas];

    for(t1 = 0; t1 < NCOMP; t1++)
    {
        // Anwendung der Logarithmusgleichung 
        if(piigN2  - piN2[t1] && figN2[curgas])
        {
          xN2 = -1 * ((airp / bN2[t1] + aN2[t1] - piN2[t1]) / (piigN2 - piN2[t1]) - 1);

            if(xN2 > 0) // Ist Logarithmieren moeglich? 
            {
                te = -1 * log(xN2) / log(2) * t05N2[t1];
                if(te < t0min)
                    t0min = te;
                calcok = 1;
            }
        }
    }

    if(calcok && dp > 10)
    {
        if(t0min > 0)
            return (int) t0min;
        else
            return 0;
   }
    else
    {
        return -1;
    }
}


// Dekompressionsstufen berechnen 
void calc_deco()
{
    float piN2x[NCOMP];
    float pambtol, pambtolmax = 1.0;
    unsigned int decostep, deco_minutes1 = 0;
    unsigned char xpos = 0, t1, t2;
    unsigned char tmp_decotime[MAX_DECO_STEPS];
    unsigned int cnt = 0;
    int ndt;

    for(t1 = 0; t1 < MAX_DECO_STEPS; t1++)
        tmp_decotime[t1] = 0;

    deco_minutes_total = 0;

    // Signal LED ein 
    led(2, 1);

    // Aktuelle Gasspannungen in temporaeres eindimensionales Datenfeld uebertragen 
    for(t1 = 0; t1 < NCOMP; t1++)
    {
        piN2x[t1] = piN2[t1];
        //pigx[t1] = piN2x[t1] + piHex[t1];
    }

    // Erste Dekostufe 
    for(t1 = 0; t1 < NCOMP; t1++)
        if((piN2x[t1] - aN2[t1]) * bN2[t1] > pambtolmax)
            pambtolmax = (piN2x[t1] - aN2[t1]) * bN2[t1];

    decostep = get_water_depth(pambtolmax);
    decostep = ((decostep / 3) + 1) * 3;

    deepest_decostep = 0;

    if(dphase)
        lcd_linecls(1, 15);

    get_dsensor();

    // Nachfolgende Dekostufen bis 0 m Wassertiefe errechnen 
    while(decostep > 0)
    {
        pambtolmax = 0.0;

        for(t1 = 0; t1 < NCOMP; t1++)
        {
            piN2x[t1] += ((get_water_pressure(decostep) - 0.0627) * figN2[curgas]  - piN2x[t1]) * (1 - exp((-1 / t05N2[t1]) * log(2)));
            pambtol = (piN2x[t1] - aN2[t1]) * bN2[t1];
            if(pambtol > pambtolmax)
                pambtolmax = pambtol;
        }

        if(get_water_depth(pambtolmax) < decostep - 3)
        {
            if(deco_minutes1)
                xpos += lcd_putnumber(1, xpos, deco_minutes1, -1, -1, 'l', 1) + 1;

            deco_minutes_total += deco_minutes1;

            // Werte im Datenfeld speichern fuer EEPROM-Aufzeichnung       
            cnt = (decostep / 3) - 1; // Nr. des Decostopp ermitteln 
            if(cnt >= 0 && cnt < MAX_DECO_STEPS)
                tmp_decotime[cnt] = deco_minutes1;

            decostep -= 3;
            deco_minutes1 = 0;
        }
        deco_minutes1 += 1;

        // Laengste gesamte Dekozeit speichern 
        if(deco_minutes_total > tmp_decotime_total)
        {
            for(t2 = 0; t2 < MAX_DECO_STEPS; t2++)
                rcd_decotime[t2] = tmp_decotime[t2];
            tmp_decotime_total = deco_minutes_total;
        }

        // Tiefsten errechneten Dekostopp speichern 
        if(decostep > deepest_decostep)
            deepest_decostep = decostep;
    }

    if(dphase || deco_minutes_total) // Restliche Anzeige (Gesamtdekozeit bzw. Nullzeit nur, wenn getaucht wird) 
    {
        if(!deco_minutes_total)      // Gesamte Dekozeit <= 0 also NZ-TG 
        {
            lcd_putstring(1, 0, "NZ: ");

            ndt = calc_ndt();

            if(ndt < 0)       // Unplausible NZ-Werte abfangen 
                lcd_putstring(1, 4, "-");
            else
            {
                xpos = lcd_putnumber(1, 4, ndt, -1, -1, 'l', 1) + 4;
                lcd_putchar(1, xpos, 39);
            }
        }
        else  // Dekompressionsstopps sind erforderlich 
        {
            // Summe der Dekozeiten anzeigen 
            lcd_putchar(1, xpos++, 246);    // Sigma-Zeichen 
            lcd_putchar(1, xpos++, '=');
            xpos += lcd_putnumber(1, xpos, deco_minutes_total, -1, -1, 'l', 0);
            lcd_putchar(1, xpos, 39);

            if(!ndt_runout) // Flag setzen fuer Profilaufzeichnung: Nullzeit zu Ende,  
            {               // PADIes muessen jetzt auftauchen! ;-P                     
                eeprom_store_byte(225);
                ndt_runout = 1;
            }

        }
    }

    if(xpos < 12)
        showtemp();

    led(2, 0);
}

// Flugverbotszeit für N2-Kompartimente berechnen 
// Aufloesung: 1 h              
unsigned int calc_no_fly_time()
{
    float piN2_b[NCOMP];
    float p_amb_tol;
    unsigned int nft = 0, flag_no_fly, t1;

    // Aktuelle Gasspannungen in temporaeres Datenfeld uebertragen 
    for(t1 = 0; t1 < NCOMP; t1++)
       piN2_b[t1] = piN2[t1];

    while(nft < 48)
    {
        flag_no_fly = 0;

        for(t1 = 0; t1 < NCOMP; t1++)
        {
            piN2_b[t1] += ((airp - 0.0627) * 0.78 - piN2_b[t1]) * (1 - exp((-60 / t05N2[t1]) * log(2)));

            p_amb_tol = (piN2_b[t1] - aN2[t1]) * bN2[t1];
            if(p_amb_tol > cabinp) // Kabinendruck in bar 
                flag_no_fly = 1;
        }

        if(!flag_no_fly)
            return nft;
        nft++;
    }
    return nft;
}

// Uebersaettigungstoleranzen veraendern 
void set_ab_values(int k, unsigned char showmode)
{
    unsigned char t1;
    double f = k * 0.1;

    for(t1 = 0; t1 < NCOMP; t1++)
    {
        aN2[t1] = 2 * exp(-0.33333333 * log(t05N2[t1]));
        //aHe[t1] = 2 * exp(-0.33333333 * log(t05He[t1]));
        bN2[t1] = 1.005 - exp(-0.5 * log(t05N2[t1]));
        //bHe[t1] = 1.005 - exp(-0.5 * log(t05He[t1]));
    }

    for(t1 = 0; t1 < NCOMP; t1++)
    {
        aN2[t1] /= f;
        bN2[t1] *= f;
    }

    if(!showmode)
        return;

    // A- und B-Werte anzeigen 
    lcd_putstring(0, 0, "a- und b-Werte:");
    wait_ms(1000);
    lcd_cls();

    for(t1 = 0; t1 < NCOMP; t1++)
    {
        lcd_putchar(0, 0, 'a');
        lcd_putnumber(0, 1, t1, -1, -1, 'l', 1);
        lcd_putnumber(0, 4, aN2[t1 + 1] * 10000, 5, 4, 'l', 1);
        lcd_putchar(1, 0, 'b');
        lcd_putnumber(1, 1, t1, -1, -1, 'l', 1);
        lcd_putnumber(1, 4, bN2[t1 + 1] * 10000, 5, 4, 'l', 1);
        wait_ms(1000);
        lcd_cls();
    }
}


// Drucksensor auslesen 
void get_dsensor()
{
    unsigned char xpos;

    adc_mode = 0;

    // AD-Wandler 
    ADMUX = 64 + 128;     // Interne Referenz auf 2,56V und Kanal 0 aktivieren PA0 PIN 40 
    wait_ms(ADWAITSTATE);
    ADCSRA = 206;   // AD-Wandler abfragen 
    wait_ms(ADWAITSTATE);

   if(depth > 999)
    {
        depth = 999;
    }

    if(depth > maxdepth)
    {
        maxdepth = depth;
        temp_maxdepth = temp;
    }


    lcd_printdiveinfo(depth, maxdepth, diveseconds * 0.0166666667);

    if(depth < (deepest_decostep - 1) * 10)
    {
        led(3, 1);
        lcd_putstring(0, 6, "!  ");
        xpos = lcd_putnumber(0, 7, deepest_decostep, -1, -1, 'l', 1) + 7;
        lcd_putstring(0, xpos, "m! ");
        wait_ms(50);
        led(3, 0);

        if(!decostep_skipped) // Flag fuer Profilaufzeichnung setzen 
        {
            eeprom_store_byte(223);
            decostep_skipped = 1;
        }
    }
}

// Temperatursensor auslesen 
void get_tsensor()
{

    adc_mode = 1;

    // AD-Wandler 
    ADMUX = 64 + 128 + 1; // Interne Referenz 2,56V und Kanal 1 aktivieren PA PIN 39 
    wait_ms(ADWAITSTATE);
    ADCSRA = 206;   // AD-Wandler abfragen 
    wait_ms(ADWAITSTATE);
}

// Akkuspannung messen
void get_vsensor()
{
    adc_mode = 2;

    // AD-Wandler 
    ADMUX = 64 + 128 + 2; // Interne Referenz und Kanal 2 aktivieren PA PIN 38 
    wait_ms(ADWAITSTATE);
    ADCSRA = 206;   // AD-Wandler abfragen 
    wait_ms(ADWAITSTATE);
}


// ppO2 (Rueckgabe = 10facher Wert!) 
int calc_ppo2(char display_warning)
{
    float fppO2   = (depth / 100 + airp) * (1 - figN2[curgas]);

   int ippO2 = (int) (fppO2 * 10);

    if(ippO2 > maxppo2)
    {
        if(display_warning)
        {
            lcd_putstring(1, 10, " ppO2!");
      }
        if(!ppo2_exceeded)
        {
            eeprom_store_byte(224);
            ppo2_exceeded = 1;
        }
    }


    return ippO2;
}


// ZNS- und OTU-Werte berechnen (Aufruf 1 x pro Minute) 
void calc_cns_otu()
{
    // ZNS-Tabelle 
    unsigned int f_day[11] =  {720, 570, 450, 360, 300, 270, 240, 210, 180, 165, 150};
    unsigned int f_dive[11] = {720, 570, 450, 360, 300, 240, 210, 180, 150, 120, 45};

    // Index des Tabellenwertes zu geg. ppO2 
    int ndx = calc_ppo2(0) - 6;

    // ppO2-Rechnungen 
    float otu_ppO2 = calc_ppo2(0) * .1 - .5;
    float cns_ppO2 = calc_ppo2(0) * .1;

    // ZNS in Oberflaechenmodus t1/2 = 90 min. 
    if(!dphase)
    {
        cns_day *= 0.992327946262943;
        return;
    }

    if(ndx >= 0 && ndx <= 10) // Normaler ppO2 => Berechnung der Dosis auf Basis der Tabelle
    {
        cns_day += 100 * exp(-1 * log(f_day[ndx]));
        cns_dive += 100 * exp(-1 * log(f_dive[ndx]));
    }

    if(ndx > 10) // Sehr hoher ppO2 => Berechnung der Dosis auf funktionaler Basis 
    {
        cns_day +=   100 * exp(-1 * log((cns_ppO2 * 432) - (cns_ppO2 - .6) * 120));
        cns_dive += 100 * exp(-1 * log((cns_ppO2 * 432) - (cns_ppO2 - .6) * 120));
    }

    // OTU 
    if(otu_ppO2 > 0)
        otu += exp(0.83 * log(otu_ppO2 * 2));
}

//***************
// Ende Dekoteil 
//***************

//************************************
// Funktionen und Prozeduren fuer LCD 
//************************************
// Ein Byte (Befehl bzw. Zeichen) zum Display senden 
void lcd_write(char lcdmode, unsigned char value, int waitcycles)
{
    set_e(0);

    if(!lcdmode)
        set_rs(0);    // RS=0 => Befehl 
    else
        set_rs(1);    // RS=1 => Zeichen 

    wait_ms(waitcycles * 2);

    set_e(1);
    PORTD = value & 0xF0;           // Hi byte 
    set_e(0);
    set_e(1);
    PORTD = (value & 0x0F) * 0x10;  // Lo byte 
    set_e(0);

}

// Ein Zeichen (Char) zum Display senden, dieses in 
// Zeile row und Spalte col positionieren           
void lcd_putchar(int row, int col, unsigned char ch)
{
    lcd_write(LCD_INST, col + 128 + row * 0x40, 1);
    lcd_write(LCD_DATA, ch, 1);
}


// Eine Zeichenkette direkt in das LCD schreiben 
// Parameter: Startposition, Zeile und Pointer   
void lcd_putstring(int row, int col, char *s)
{
    unsigned char t1;

    for(t1 = col; *(s); t1++)
        lcd_putchar(row, t1, *(s++));
}


// Display loeschen 
void lcd_cls(void)
{
    lcd_write(LCD_INST, 1, 5);
}


// Display loeschen (eine Zeile) 
void lcd_linecls(int displine, int chars)
{
    unsigned char t1;

    for(t1 = 0; t1 <= chars; t1++)
        lcd_putchar(displine, t1, 32);
}

// E setzen 
void set_e(char status)  // PORT C0 = Pin 6 am LCD 
{
    if(status)
   {
        //sbi(PORTC, 0);
      PORTC |= _BV(PC0);
   }
    else
   {
        //cbi(PORTC, 0);
      PORTC &= ~_BV(PC0);
   }
}

// RS setzen 
void set_rs(char status) // PORT C1 = Pin 4 am LCD 
{
    if(status)
   {
        //sbi(PORTC, 1);
      PORTC |= _BV(PC1);
   }
   else
   {
        //cbi(PORTC, 1);
      PORTC &= ~_BV(PC1);
   }
}

// LCD-Display initialisieren 
void lcd_init(void)
{
    // Grundeinstellungen: 2 Zeilen, 5x7 Matrix, 4 Bit 
    lcd_write(LCD_INST, 40, 5);
    lcd_write(LCD_INST, 40, 5);
    lcd_write(LCD_INST, 40, 5);

    lcd_write(LCD_INST, 2, 5);
    lcd_write(LCD_INST, 8, 5);

    // Display on, Cursor off, Blink off 
    lcd_write(LCD_INST, 12, 5);

    lcd_cls();

    // Entrymode !cursoincrease + !displayshifted 
    lcd_write(LCD_INST, 4, 5);
}


// Eine n-stellige Zahl direkt in das LCD schreiben 
// Parameter: Startposition und Zeile; Zahl,        
// darzustellende Ziffern, Position des Dezimalpunktes, (l)links- oder (r)echtsbuendig 
int lcd_putnumber(int row, int col, int num, int digits, int dec, char orientation, char lead0)
{
    char cl = col, minusflag = 0;
    unsigned char cdigit[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, digitcnt = 0;
    int t1, t2, n = num, r, x = 1;

    if(num < 0)
    {
        minusflag = 1;
        n *= -1;
    }

    // Stellenzahl automatisch bestimmen 
    if(digits == -1)
    {
        for(t1 = 1; t1 < 10 && (n / x); t1++)
            x *= 10;
        digits = t1 - 1;
    }

    if(!digits)
        digits = 1;

    for(t1 = digits - 1; t1 >= 0; t1--)
    {
        x = 1;
        for(t2 = 0; t2 < t1; t2++)
            x *= 10;
        r = n / x;
        cdigit[digitcnt++] = r + 48;

        if(t1 == dec)
            cdigit[digitcnt++] = 46;
        n -= r * x;
    }

   //Fuehrende Nullen abschneiden falls lead0 == 0
   if(!lead0)
   {
       t1 = 0;
       while(cdigit[t1])
        {
           if(cdigit[t1] == '0' && cdigit[t1 + 1] != '.' && cdigit[t1 + 1] != 0)
          {
              cdigit[t1] = ' ';
         }
            t1++;
        }
   }

    digitcnt--;
    t1 = 0;

    // Ausgabe 
    switch(orientation)
    {
      case 'l':
        cl = col;
        if(minusflag)
        {
            lcd_putchar(row, cl++, '-');
            digitcnt++;
        }

        while(cl <= col + digitcnt)                       // Linksbuendig 
            lcd_putchar(row, cl++, cdigit[t1++]);

        break;

      case 'r':
        t1 = digitcnt;                              // Rechtsbuendig 
        for(cl = col; t1 >= 0; cl--)
            lcd_putchar(row, cl, cdigit[t1--]);
        if(minusflag)
            lcd_putchar(row, --cl, '-');
    }

    if(dec == -1)
        return digits;
    else
        return digits + 1;
}

// Alle Daten des laufenden TG an die richtigen Stellen des LCD  
// schreiben: Parameter: Tiefen in m, Zeit in sec.               
void lcd_printdiveinfo(int cdepth, int mdepth, int divetime)
{
    lcd_putnumber(0, 0, cdepth, 3, 1, 'l', 0);
    lcd_putchar(0, 4, 'm');

    lcd_putnumber(0, 6, mdepth, 3, 1, 'l', 0);
    lcd_putstring(0, 10, "m");

    lcd_putnumber(0, 14, divetime, -1, -1, 'r', 1); // Tauchzeit rechtsbuendig 1. Zeile 
    lcd_putchar(0, 15, 39);
}


// Temperatur anzeigen 
void showtemp()
{
    lcd_putstring(1, 12, "   ");
    lcd_putnumber(1, 14, temp * 10, 3, 1, 'r', 0); // ORIG!
   //lcd_putnumber(1, 14, temp, -1, -1, 'r', 0); //TEST zur Ausgabe des ADC-Wertes!

   lcd_putchar(1, 15, 223); // °-Zeichen  // ORIG!
}

void show_accu_voltage()
{
    get_vsensor();
    lcd_putstring(1, 0, "BAT:");
    lcd_putchar(1, 5 + lcd_putnumber(1, 5, accu_voltage * 10, 2, 1, 'l', 1), 'V');
   //lcd_putnumber(1, 5, accu_voltage, -1, -1, 'l', 1); //TEST
}

// Aktuelles Atemgas anzeigen 
void show_gas(int gasnum)
{
    unsigned char xpos;

    lcd_cls();
    if(gasnum < MAXGASES)
    {
        lcd_putstring(0, 0, "Gas");
        lcd_putnumber(0, 4, gasnum + 1, -1, -1, 'l', 1);

        lcd_putstring(1, 0, "N2:");
        xpos = lcd_putnumber(1, 4, figN2[gasnum] * 100, -1, -1, 'l', 1) + 4;
        lcd_putchar(1, xpos, '%');
    }
}

// Aktuelles Atemgas einstellen 
void set_curgas()
{
    unsigned char lcurgas = curgas;
    unsigned long lseconds = runseconds;

    show_gas(lcurgas);

    while(get_keys());

    do
    {
        if(get_keys() == 3)
        {
            lseconds = runseconds;
            lcurgas++;
            if(lcurgas >= MAXGASES)
                lcurgas = 0;
            show_gas(lcurgas);
            while(get_keys() == 3);
        }
    }
    while(runseconds < lseconds  + 5);

    lcd_cls();

    if(curgas != lcurgas)
    {
        lcd_putstring(0, 0, "Wechsel zu Gas");
        lcd_putnumber(0, 15, lcurgas + 1, -1, -1, 'l', 1);
        curgas = lcurgas;
        eeprom_store_byte(226);
        eeprom_store_byte(curgas);
    }

    wait_ms(2000);
    lcd_cls();
}


// Tiefenwert runden auf 1 Nachkommastelle u. /= 10 
int round_depth(int d)
{
    if(d - (d / 10) * 10 >= 5)
        return (d / 10 + 1);
    else
        return (d / 10);
}

void calc_airp_divesite(char show_mode)
{
    char xpos;
    int airp0_tmp = airp0 * 1000;

    airp0_tmp = airp0_tmp * exp(-1.2928 * 0.0981 * altitude / airp0_tmp);

    if(show_mode)
    {
        wait_ms(INITWAIT);
        lcd_cls();
        lcd_putstring(0, 0, "Luftdruck TP");
        xpos = lcd_putnumber(1, 0, airp0_tmp, -1, -1, 'l', 1) + 1;
        lcd_putstring(1, xpos, "mbar");
    }

    airp = airp0_tmp * 0.001;
}
// Ende LCD-Teil 

//*******
// USART 
//*******
void usart_init()
{
    // 2.4 kBaud 
    UBRRL = 220; // Originaler Wert = 207 fuer 2.4k 215 
    UBRRH = 0;

    // RX Interrupt, RX und TX einschalten 
    UCSRB = (1<<RXCIE)|(1<<RXEN)|(1<<TXEN);

    // 8 Datenbits, 1 Stopbit, keine Paritaet 
    UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);

    rx_buf_cnt = 0;
}

void usart_putc(char tx_char)
{
    while(!(UCSRA & (1<<UDRE)));

    UDR = tx_char;
}

SIGNAL(SIG_UART_RECV)
{
    unsigned char rx_char = UDR, inputlen = 2;
    unsigned int t1, byte_adr, x = 0;;

    if(rx_buf_cnt < RX_BUF_SIZE)
    {
        rx_buf[rx_buf_cnt] = rx_char;

        switch(rx_buf[0])
        {
          case 100:
            inputlen = 2; // 1 Byte lesen 
            rx_buf_cnt++;
            break;

          case 101:
            inputlen = 4; // 1 Byte schreiben 
            rx_buf_cnt++;
            break;

          default:   clear_rx_buf();
        }
    }
    else
        clear_rx_buf();

    if(rx_buf_cnt > inputlen)
    {
        // Befehlsfolge quittieren 
        for(t1 = 0; t1 < rx_buf_cnt; t1++)
            usart_putc(rx_buf[t1]);

        byte_adr = rx_buf[1] + rx_buf[2] * 256;

        if(byte_adr <= MAX_EEPROM_ADR)
        {
            lcd_putnumber(1, 8, byte_adr, 4, -1, 'l', 1);

            // Code auswerten 
            switch(rx_buf[0])
            {
              case 100:  // 1 Byte lesen 
                usart_putc(eeprom_read_byte((uint8_t*)byte_adr));               // Byte senden 
                usart_putc(make_crc(3, eeprom_read_byte((uint8_t*)byte_adr)));  // CRC anhaengen 
                lcd_putstring(1, 0, "Tx  ");
                lcd_putnumber(1, 13, eeprom_read_byte((uint8_t*)byte_adr), 3, -1, 'l', 1);
                break;

              case 101:  // 1 Byte schreiben 
                for(t1 = 0; t1 < 4; t1++)  // CRC berechnen 
                x = x ^ rx_buf[t1];

            if(x == rx_buf[4])   // CRC ist OK 
            {
                cli();
                    while(!eeprom_is_ready());
                    eeprom_write_byte((uint8_t*)(rx_buf[1] + rx_buf[2] * 256), rx_buf[3]);
                    sei();
                    lcd_putstring(1, 0, "Rx  ");
                    lcd_putnumber(1, 13, rx_buf[3], 3, -1, 'l', 1);
            }
                else
                {
                lcd_putstring(1, 0, "CRC!");
            }
            }
        }
        clear_rx_buf();
    }
}

// CRC-Pruefsumme berechnen 
char make_crc(int buflen, int addchar)
{
    int t1, x = 0;

    for(t1 = 0; t1 < buflen; t1++) // Puffer bis dato 
        x = x ^ rx_buf[t1];
    x = x ^ addchar;                // Sendebyte 

    return x;
}

// Empfangspuffer loeschen 
void clear_rx_buf()
{
    int t1;

    for(t1 = 0; t1 < RX_BUF_SIZE; t1++)
        rx_buf[t1] = 0;
    rx_buf_cnt = 0;
}

// Hauptfunktion der Schnittstelle zwischen SBTC und PC 
void sbtc2pc()
{

    while(get_keys());
    lcd_cls();

    // Datenuebertragung zum PC starten? 
    lcd_putstring(0, 0, "SBTC <-> PC?");
    lcd_putstring(1, 0, "(j/n)");
    do
    {
        if(get_keys() == 3)
        {
            usart_init();
            lcd_cls();
            lcd_putstring(0, 0, "Modus");
            lcd_putstring(0, 8, "ADRS VAL");

            while(get_keys() != 2);
            UCSRB = 0;
            return;
        }
    }while(get_keys() != 2);
    while(get_keys());
    lcd_cls();

}

void display_profile()
{
    unsigned int t1;
   int startbyte = 55, endbyte;
   int xdepth = 0;
    unsigned char xpos, ok, p_cnt = 1;

    while(get_keys());
    lcd_cls();

    lcd_putstring(0, 0, "TG-Profil an-");
    lcd_putstring(1, 0, "zeigen? (j/n)");

    do
    {
        if(get_keys() == 3)
        {
          cli();
         while(1)
         {
            lcd_cls();

            // Startbyte finden 
            t1 = startbyte;
            endbyte = 0;
            ok = 0;
            while(t1 < MAX_EEPROM_ADR && !ok)
            {
               if(eeprom_read_byte((uint8_t*)t1) == 229)
               {
                  startbyte = t1;
                  ok = 1;
               }
               else
                  t1++;
            }

            t1 = startbyte + 1;
            ok = 0;
            while(t1 < MAX_EEPROM_ADR && !ok)
            {
               if(eeprom_read_byte((uint8_t*)t1) == 230)
               {
                  endbyte = t1;
                  ok = 1;
               }
               else
                  t1++;
            }

            if(endbyte)
            {
               lcd_cls();
               lcd_putstring(0, 0, "Profil");
               lcd_putnumber(0, 7, p_cnt++, -1, -1, 'l', 1);
               wait_ms(1000);

               lcd_cls();
               lcd_putstring(0, 0, "Zeit");
               lcd_putstring(0, 8, "Tiefe");
               for(t1 = startbyte + 1; t1 < endbyte; t1++)
               {
                  xdepth = eeprom_read_byte((uint8_t*)t1);
                  if(xdepth < 99)
                  {
                     lcd_linecls(1, 15);

                     xpos = lcd_putnumber(1, 0, (int)((t1 - startbyte) * 0.3333333), -1, -1, 'l', 1) + 1;
                     lcd_putstring(1, xpos, "min.");


                     xpos = lcd_putnumber(1, 8, xdepth, -1, -1, 'l', 1) + 9;
                     lcd_putchar(1, xpos, 'm');
                     wait_ms(500);

                     if(get_keys() == 2)
                     {
                        lcd_cls();
                        sei();
                        return;
                     }
                  }
               }
            }
            else
            {
               lcd_cls();
               lcd_putstring(0, 0, "Keine (weiteren)");
               lcd_putstring(1, 0, "Profile.");
               wait_ms(2000);
               lcd_cls();
               sei();
               return;
            }
            startbyte = endbyte + 1;
            endbyte = 0;
         }
      }
   }while(get_keys() != 2);
   while(get_keys());
   lcd_cls();
}

// Werte für Gesamttauchzeit, Max. Tiefe etc. 
void display_rcd()
{
    unsigned char xpos;
   unsigned long dminutes_t;
   unsigned int dminutes, dhours;

    lcd_putstring(0, 0, "Logwerte zeigen?");
   lcd_putstring(1, 0, "(j/n)");

    do
    {
        if(get_keys() == 3)
        {
            lcd_cls();
         lcd_putstring(0, 0, "Anzahl TG:");
         lcd_putnumber(1, 0, eeprom_read_byte((uint8_t*)24) + eeprom_read_byte((uint8_t*)25) * 256 + 1, -1, -1, 'l', 1);
         while(get_keys() != 2);
         while(get_keys());

         lcd_cls();
         lcd_putstring(0, 0, "Ges. Tauchzeit:");

         dminutes_t = eeprom_read_byte((uint8_t*)26) + eeprom_read_byte((uint8_t*)27) * 256;
         dhours = dminutes_t / 60;
         dminutes = dminutes_t - dhours * 60;

         xpos = lcd_putnumber(1, 0, dhours, -1, -1, 'l', 1) + 1;
         lcd_putstring(1, xpos, "Std.");
         xpos = lcd_putnumber(1, 8, dminutes, -1, -1, 'l', 1) + 9;
         lcd_putstring(1, xpos, "Min.");
            while(get_keys() != 2);
         while(get_keys());

         lcd_cls();
         lcd_putstring(0, 0, "Max. Tiefe:");
         xpos = lcd_putnumber(1, 0, eeprom_read_byte((uint8_t*)28) + eeprom_read_byte((uint8_t*)29) * 256, 3, 1, 'l', 1) + 1;
         lcd_putstring(1, xpos, "m");
      }
   }while(get_keys() != 2);
   while(get_keys());
   lcd_cls();
}


void display_log()
{

    unsigned int t1;
   int startbyte = 55;
    unsigned char xpos, ok, p_cnt = 1;

    while(get_keys());
    lcd_cls();

    lcd_putstring(0, 0, "TG-Daten an-");
    lcd_putstring(1, 0, "zeigen? (j/n)");

    do
    {
        if(get_keys() == 3)
        {
          cli();
         while(1)
         {
            lcd_cls();

            // Anfang eines Datensatzes im EEPROM-Speicher suchen 
            t1 = startbyte;
            ok = 0;
            while(t1 < MAX_EEPROM_ADR && !ok)
            {
               if(eeprom_read_byte((uint8_t*)t1) == 230)
               {
                  startbyte = t1;
                  ok = 1;
               }
               else
                  t1++;
            }

            if(startbyte && ok)
            {
               lcd_cls();
               lcd_putstring(0, 0, "TG Nr.");
               lcd_putnumber(0, 7, p_cnt++, -1, -1, 'l', 1);
               wait_ms(1000);

               lcd_cls();

               lcd_putstring(0, 0, "Tauchzeit in Min.");
               lcd_putnumber(1, 0, eeprom_read_byte((uint8_t*)startbyte + 1) + eeprom_read_byte((uint8_t*)startbyte + 2) * 256, -1, -1, 'l', 1);
               wait_ms(1000);
               lcd_cls();

                  lcd_putstring(0, 0, "Max. Tiefe in m");
               lcd_putnumber(1, 0, (eeprom_read_byte((uint8_t*)startbyte + 3) + eeprom_read_byte((uint8_t*)startbyte + 4) * 256) / 10, -1, -1, 'l', 1);
               wait_ms(1000);
               lcd_cls();

               lcd_putstring(0, 0, "Dekostufen");

               ok = 0;
               t1 = startbyte + 5;
               xpos = 0;
               while(t1 < MAX_EEPROM_ADR && t1 < startbyte + 10 && !ok)
                {
                   if(eeprom_read_byte((uint8_t*)t1++) == 231)
                   {
                      ok = 1;

                     while(t1 < MAX_EEPROM_ADR && ok && eeprom_read_byte((uint8_t*)t1++) != 232)
                        xpos = lcd_putnumber(1, xpos, eeprom_read_byte((uint8_t*)t1), -1, -1, 'l', 1) + 2;
                   }
                }
            }
          }
      }
    }while(get_keys() != 2);
   while(get_keys());
   lcd_cls();

}

// Flash-Speicher löschen 
// Parameter: 1: Nur Profilspeicher löschen, 2: kompletten Speicher löschen 
void clear_flash(char erasemode)
{
    unsigned int t1, startadr = 0;

    while(get_keys());
    lcd_cls();

   if(erasemode == 1)
       startadr = EEPROM_PROF_START;

    // TG-Profildaten loeschen? 
   if(startadr == EEPROM_PROF_START)
   {
        lcd_putstring(0, 0, "TG-Profile loe-");
        lcd_putstring(1, 0, "schen? (j/n)");
    }
   else
   {
        lcd_putstring(0, 0, "Flashspeicher");
        lcd_putstring(1, 0, "loeschen? (j/n)");
   }

    do
    {
        if(get_keys() == 3)
        {
            lcd_cls();
            lcd_putstring(0, 0, "Loesche Byte:");
            cli();
            for(t1 = startadr; t1 <= MAX_EEPROM_ADR; t1++)
            {
                while(!eeprom_is_ready());
                eeprom_write_byte((uint8_t*)t1, 0);
                lcd_putnumber(1, 0, t1, -1, -1, 'l', 1);
            }
            while(!eeprom_is_ready());
            eeprom_write_byte((uint8_t*)30, EEPROM_PROF_START);
            while(!eeprom_is_ready());
            eeprom_write_byte((uint8_t*)31, 0);
            sei();
            lcd_cls();
            return;
        }
    }while(get_keys() != 2);
    while(get_keys());
    lcd_cls();

}

// Wartezeit in Millisekunden bei fck = 8.000 MHz 
//Warteschleife in Millisekunden
void wait_ms(int ms)
{
   unsigned int t1;

    for(t1 = 1; t1 < ms; t1++)
      _delay_ms(1);
}

// LEDs an PC2, 3 und 4 
void led(char lednum, char status)
{
    switch(lednum)
   {
       case 2: lednum = PC2;
              break;
       case 3: lednum = PC3;
              break;
       case 4: lednum = PC4;
              break;
   }

    if(status)
   {
        //cbi(PORTC, lednum);
      PORTC &= ~_BV(lednum);
   }
    else
   {
        //sbi(PORTC, lednum);
      PORTC |= _BV(lednum);
   }
}


// Einstellungen durch Benutzer                
// Tastenentprellung per Hardware erforderlich 
int get_keys(void)
{
    int t1;
   for(t1 = 0; t1 < 3;t1++)
     if(!bit_is_set(PINB, t1))
        return (t1 + 1);
   return 0;
}

// Benutzereinstellungen 
void settings(void)
{
    int menu_sta[MENU_ITEMS] = {900, 0, 400, 10, 3, 0, 0};         // Startwerte fuer Wertepektrum 
    int menu_end[MENU_ITEMS] = {1100, 4000, 1000, 20, 20, 1, 1};   // Endwerte fuer Wertepektrum   
    int menu_step[MENU_ITEMS] = {5, 100, 5, 1, 1, 1, 1};           // Inkrement                    

    int menu_N2[3]; // Temporaere Werte fuer Stickstoff 

    int intv, t1;

    char ch, xpos;

    int menu_tmpval[MENU_ITEMS];
    menu_tmpval[0] = airp0 * 1000;   // Luftdruck                                                     
    menu_tmpval[1] = altitude;       // Hoehe ueber NN                                                
    menu_tmpval[2] = cabinp * 1000;  // Kabinendruck Flugzeug                                         
    menu_tmpval[3] = maxppo2;        // Max. zul. Sauerstoffpartialdruck (10facher Wert!)             
    menu_tmpval[4] = f_cons;         // Multiplikationsfaktor fuer Übersaettigungstoleranzen          
    menu_tmpval[5] = eeprom_read_byte((uint8_t*)18);  // ppN2 nach TG-Ende anzeigen
   menu_tmpval[6] = eeprom_read_byte((uint8_t*)19);  // Beim Starten Einstellungen anzeigen?

    for(t1 = 0; t1 < MAXGASES; t1++)
    {
        menu_N2[t1] = figN2[t1] * 100;
    }

    while(get_keys());
    lcd_cls();

    for(t1 = 0; t1 < MENU_ITEMS; t1++)
    {
        lcd_putstring(0, 0, menu_str[t1]);
        xpos = lcd_putnumber(1, 0, menu_tmpval[t1], menu_digits[t1], menu_dec[t1], 'l', 1) + 1;
        lcd_putstring(1, xpos, menu_unitstr[t1]);

        do
        {
            ch = get_keys();
            if(ch == 1 || ch == 3)
            {
                lcd_linecls(1, 15);
                intv =  menu_tmpval[t1] / menu_step[t1];
                switch(ch)
                {
                  case 3:
                    menu_tmpval[t1] = intv * menu_step[t1] + menu_step[t1];
                    if(menu_tmpval[t1] > menu_end[t1])
                        menu_tmpval[t1] = menu_sta[t1] ;
                    break;

                  case 1:
                    menu_tmpval[t1]  = intv *  menu_step[t1] - menu_step[t1];
                    if(menu_tmpval[t1] < menu_sta[t1])
                        menu_tmpval[t1] = menu_end[t1];
                }
                xpos = lcd_putnumber(1, 0, menu_tmpval[t1], menu_digits[t1], menu_dec[t1], 'l', 1) + 1;
                lcd_putstring(1, xpos, menu_unitstr[t1]);
                wait_ms(100);
            }
        }while(ch != 2);
        while(get_keys());
        lcd_cls();
    }

    // 3 Atemgase 
    for(t1 = 0; t1 < MAXGASES; t1++)
    {
        // N2 
        lcd_putstring(0, 0, "Gas   N2-Anteil");
        lcd_putnumber(0, 4, t1 + 1, -1, -1, 'l', 1);
        xpos = lcd_putnumber(1, 0, menu_N2[t1], -1, -1, 'l', 1);
        lcd_putstring(1, xpos, "%  ");
        do
        {
            ch = get_keys();
            if(ch == 1 || ch == 3)
            {
                lcd_linecls(1, 15);
                switch(ch)
                {
                  case 3:
                    menu_N2[t1]++;
                    if(menu_N2[t1] > 79)
                        menu_N2[t1] = 0;
                    break;

                  case 1:
                    menu_N2[t1]--;
                    if(menu_N2[t1] < 0)
                        menu_N2[t1] = 79;
                }
                xpos = lcd_putnumber(1, 0, menu_N2[t1], -1, -1, 'l', 1);
                lcd_putstring(1, xpos, "%  ");
                lcd_putstring(1, 4, "(Nitrox");
                xpos = lcd_putnumber(1, 12, 100 - menu_N2[t1], -1, -1, 'l', 1) + 12;
            lcd_putstring(1, xpos, ")");
            }
            wait_ms(100);
        }while(ch != 2);
        while(get_keys());
        lcd_cls();
    }

    // Speichern? 
    lcd_putstring(0, 0, "Sichern? (j/n)");
    do
    {
        if(get_keys() == 3)
        {
            cli();
            // Luftdruck am Tauchort 
            airp0 = menu_tmpval[0] * 0.001;
            while(!eeprom_is_ready());
            eeprom_write_byte((uint8_t*)0, menu_tmpval[0] & 0x00FF);          // LoByte 
            while(!eeprom_is_ready());
            eeprom_write_byte((uint8_t*)1, (menu_tmpval[0] & 0xFF00) / 256);  // HiByte 

            // Hoehe ueber NN 
            altitude = menu_tmpval[1];
            while(!eeprom_is_ready());
            eeprom_write_byte((uint8_t*)16, menu_tmpval[1] & 0x00FF);          // LoByte 
            while(!eeprom_is_ready());
            eeprom_write_byte((uint8_t*)17, (menu_tmpval[1] & 0xFF00) / 256);  // HiByte 

            calc_airp_divesite(1); // Luftdruck am Tauchort nachberechnen 
            wait_ms(2000);
            lcd_cls();

            // Kabinendruck im Flugzeug 
            cabinp = menu_tmpval[2]* 0.001;
            while(!eeprom_is_ready());
            eeprom_write_byte((uint8_t*)14, menu_tmpval[2] & 0x00FF);          // LoByte 
            while(!eeprom_is_ready());
            eeprom_write_byte((uint8_t*)15, (menu_tmpval[2] & 0xFF00) / 256);  // HiByte 

            // max. ppO2 
            maxppo2 = menu_tmpval[3];
            while(!eeprom_is_ready());
            eeprom_write_byte((uint8_t*)11, menu_tmpval[3]);

            // Konservativfaktor 
         // Anzeigen der a- und b-Werte wenn geändert 
            if(menu_tmpval[4] != f_cons)
            {
                set_ab_values(menu_tmpval[4], 1);
                while(!eeprom_is_ready());
                eeprom_write_byte((uint8_t*)10, menu_tmpval[4]);
                f_cons = menu_tmpval[4];
            }

            // ppN2-Anzeige nach TG 
            show_ppN2 = menu_tmpval[5];
            while(!eeprom_is_ready());
            eeprom_write_byte((uint8_t*)18, menu_tmpval[5]);

         // Einstellungen beim Starten anzeigen?
            show_ppN2 = menu_tmpval[6];
            while(!eeprom_is_ready());
            eeprom_write_byte((uint8_t*)19, menu_tmpval[6]);

            // Gase 
            for(t1 = 0; t1 < MAXGASES; t1++)
            {
                figN2[t1] = menu_N2[t1] * 0.01;
                while(!eeprom_is_ready());
                eeprom_write_byte((uint8_t*)(t1 * 2 + 2), menu_N2[t1]);
            }

            sei();

            lcd_putstring(0, 2, "Gespeichert.");
         wait_ms(1000);
         lcd_cls();
         return;
        }
    }while(get_keys() != 2);
    while(get_keys());
    lcd_cls();
}

// Timer 2 Ereignisroutine (autom. Aufruf 1/s) 
ISR(SIG_OVERFLOW2)
{
    runseconds++;

    TCNT2 = 0;       // Timerregister auf 0 
}

// AD-Wandler Ereignisroutine 
SIGNAL(SIG_ADC)
{
    unsigned char lo, hi;

    lo = ADCL;
    hi = ADCH;

    adc_val = hi * 256 + lo;
   switch(adc_mode)
   {
      case 0: depth = adc_val;
     break;
     case 1: temp = (adc_val - 394.6344) / 2.9656;
             // temp = adc_val; //Test zur Ausgabe des ADC-Wertes

     break;
     case 2: accu_voltage = (double) adc_val / 69;
             //accu_voltage = adc_val; //Test zur Ausgabe des ADC-Wertes
     break;
   }
}

void eeprom_store_byte(char eeprom_val)
{
    if(eeprom_byte_count < EEPROM_PROF_START || eeprom_byte_count > MAX_EEPROM_ADR)
        eeprom_byte_count = EEPROM_PROF_START;

    cli();
    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)eeprom_byte_count++, eeprom_val);
    sei();
}

int main()
{
    unsigned long seconds_old1, seconds_old2, seconds_old3;
   unsigned long subseconds = 0;
    unsigned char do_record_depth = 0; // Intervallschalter fuer Profilaufzeichnung 
   unsigned char info_mode = 0;       // Definieren, was angezeigt werden soll     
    unsigned char nft = 0;             // Flugverbotszeit                           
   unsigned char is_deco;
    char xpos;
   unsigned int cur_comp = 0;
   char max_info_mode;
   unsigned long surf_hrs, surf_mins;

    int t1;

    // Ports einrichten 
    // OUTPUT 
    // Port D Bit 4-7 auf "Output" schalten (=Display=DB4-DB7) 
    DDRD = 0xF0;
    // Port C Bit 0 bis 4 auf "Output" schalten 
    // LCD RS, E, LEDs 
    DDRC = 0x3F;

    DDRB = 0x00;  // Port B auf Input schalten für taster 1-3
    PORTB = 0x07; //Interne Pull-up-Widerstände an PB0, PB1 und PB2 zuschalten

    // Alle LEDs aus 
    for(t1 = 2; t1 < 4; t1++)
        led(t1, 0);

    // Mit LCD-Initialisierung 0.2 s warten bis PowerUp von MC OK 
    wait_ms(200);
    lcd_init();
    lcd_putstring(0, 4, "SBTC 3b");

    // Softwareversion 
    for(t1 = 0; t1 < 3; t1++)
    {
        if(softwareversion[t1] != eeprom_read_byte((uint8_t*)21 + t1))
        {
            while(!eeprom_is_ready());
            eeprom_write_byte((uint8_t*)21 + t1, softwareversion[t1]);
        }
    }

    lcd_putstring(1, 5, "V .");
    lcd_putnumber(1, 6, softwareversion[0], -1, -1, 'l', 1);
    lcd_putnumber(1, 8, softwareversion[1], 2, -1, 'l', 1);
    lcd_putchar(1, 10, softwareversion[2]);

    // Watchdog aus, wird nicht gebraucht, da Software zuverlaessig ist ;-)) 
    // Logisch '1' in WDTOE und WDE schreiben 
    WDTCR = (1<<WDTOE) | (1<<WDE);
    // WDT abschalten 
    WDTCR = 0x00;
    wait_ms(INITWAIT * 2);

   // ppN2 anzeigen
    show_settings = eeprom_read_byte((uint8_t*)19);
    if (show_settings   != 1 && show_settings != 0)
        show_settings = 0;

   // Konservativ-Faktor auf 5 setzen (^= *= 1.2) 
    f_cons = 12;
    set_ab_values(f_cons, 0);

    if(show_settings)
   {
      // Umgebungsluftdruck 
      lcd_cls();
      airp0 = (eeprom_read_byte((uint8_t*)0) + eeprom_read_byte((uint8_t*)1) * 256) * 0.001;  // Luftdruck 
      if(airp0 < 0.66 || airp0 > 1.2)
         airp0 = 1;
      lcd_putstring(0, 0, menu_str[0]);
      xpos = lcd_putnumber(1, 0, airp0 * 1000, -1, -1, 'l', 1) + 1;
      lcd_putstring(1, xpos, menu_unitstr[0]);

      // Hoehe ueber NN 
      wait_ms(INITWAIT);
      lcd_cls();
      altitude = (eeprom_read_byte((uint8_t*)16) + eeprom_read_byte((uint8_t*)17) * 256);  // Hoehe ueber NN in m 
      if(altitude < 0 || altitude > 6000)
         altitude = 0;
      lcd_putstring(0, 0, menu_str[1]);
      xpos = lcd_putnumber(1, 0, altitude , -1, -1, 'l', 1) + 1;
      lcd_putstring(1, xpos, menu_unitstr[1]);

      // Luftdruck am Tauchort 
      calc_airp_divesite(1);

      // Kabinendruck im Flugzeug 
      wait_ms(INITWAIT);
      lcd_cls();
      cabinp = (eeprom_read_byte((uint8_t*)14) + eeprom_read_byte((uint8_t*)15) * 256) * 0.001;  // Wert aus EEPROM lesen 
      if(cabinp < 0.55 || cabinp > 1)
         cabinp = 0.75;
      lcd_putstring(0, 0, menu_str[2]);
      xpos = lcd_putnumber(1, 0, cabinp * 1000, -1, -1, 'l', 1) + 1;
      lcd_putstring(1, xpos, menu_unitstr[2]);

      // N2- und He-Anteile in den 4 Gasen 
      wait_ms(INITWAIT);
      figN2[0] = FN2;
      show_gas(0);
      wait_ms(INITWAIT);

      for(t1 = 1; t1 < MAXGASES; t1++)
      {
         figN2[t1] = eeprom_read_byte((uint8_t*)(t1 * 2 + 2)) * 0.01;
         if(figN2[t1] < 0 || figN2[t1] > 0.78)
            figN2[t1] = 0.78;

         show_gas(t1);
         wait_ms(INITWAIT);
      }

      // maxppo2 
      maxppo2 = eeprom_read_byte((uint8_t*)11);
      if(maxppo2 > 20 || maxppo2 < 10)
         maxppo2 = 16;

      lcd_putstring(0, 0, menu_str[3]);
      lcd_putnumber(1, 0, maxppo2, 2, 1, 'l', 1);
      lcd_putstring(1, 4, menu_unitstr[3]);

      wait_ms(INITWAIT);
      lcd_cls();

      // ppN2 anzeigen 
      show_ppN2 = eeprom_read_byte((uint8_t*)18);
      if (show_ppN2  != 1 && show_ppN2 != 0)
         show_ppN2  = 0;
      lcd_putstring(0, 0, menu_str[5]);
      if(show_ppN2 )
         lcd_putstring(1, 0, "an");
      else
         lcd_putstring(1, 0, "aus");

      wait_ms(INITWAIT);
      lcd_cls();

      //Konservativfaktor
      lcd_putstring(0, 0, menu_str[4]);
      lcd_putnumber(1, 0, f_cons, menu_digits[4], menu_dec[4], 'l', 1);
      wait_ms(INITWAIT);
   }

    curgas = 0;

    lcd_cls();

    // Timer 2 fuer Sekundenzaehlung initialisieren 
    // (asynchron getaktet durch 32.768 kHz-Quarz)  
    TIMSK &=~((1<<TOIE2)|(1<<OCIE2));  // Disable TC2 interrupt 
    ASSR |= (1<<AS2);                   // Timer/Counter2 auf asynchronen Betrieb mit quarz 32,768kHz schalten 
    TCNT2 = 0x00;                        // Startwert fuer Timer2 
    TCCR2 = 0x05;                        // Teiler ck/128 
    while(ASSR & 0x07);                 // Warten bis ASSR-Register neu geschrieben wurde 
    TIMSK |= (1<<TOIE2);                // Interrupt ermoeglichen 

    sei();

    seconds_old1 = runseconds - 10;
    seconds_old2 = runseconds;
    seconds_old3 = runseconds;


    for(;;) // Endlosschleife fuer period. Aufgaben (Druckmessung, Dekorechnung, etc.) Periode: 1/s 
    {
        get_dsensor();   // Sensorabfrage Drucksensor          

        calc_ppo2(1);    // ppO2 pruefen                       

        if(!depth && !surfaced && dphase)
        {
            eeprom_store_byte(227); // "Aufgetaucht" ins Log schreiben 
            surfaced = 1;
        }

        if(temp < temp_min)
            temp_min = temp;

        if(depth > SWITCHDEPTH)
        {
            surfaced = 0;
         subseconds++;

            if(!dphase && subseconds > 10) // TG beginnt wenn 10 Sekunden ausreichend abgetaucht wurde.
            {                                // => Es wird auf "Tauchphase" umgeschaltet.
                maxdepth = 0;
                diveseconds = 0;
                temp_min = 100;
                deco_minutes_total = 0;
                rcd_deco_minutes_total = 0;

            for(t1= 0; t1 < MAX_DECO_STEPS; t1++)
                rcd_decotime[t1] = 0;
                ndt_runout = 0;
                cns_dive = 0;
                temp_low = 0;

                lcd_cls();

                // Neues TG-Profil im Ringspeicher anlegen, Startpunkt suchen 
                eeprom_byte_count = eeprom_read_byte((uint8_t*)30) + 256 * eeprom_read_byte((uint8_t*)31);

                // Startsignal 
                eeprom_store_byte(228);

                // Oberflaechenpause speichern 
                eeprom_store_byte((surf_seconds / 60) & 0x00FF);          // Lo 
                eeprom_store_byte(((surf_seconds / 60) & 0xFF00) / 256);  // Hi 

                // Temperatur zu TG-Beginn 
                eeprom_store_byte(temp);

                // Aufzeichnungsintervall 
                eeprom_store_byte(20);

                // Indikator fuer den Beginn des TG-Profiles 
                eeprom_store_byte(229);

                surf_seconds = 0;

            dphase = 1;
            }

         diveseconds++;
        }
        else // Taucher an der Oberflaeche
        {
          subseconds = 0;

            if(surf_seconds > SURF_SECONDS_MAX && !deco_minutes_total)    // TG beendet 
            {
                if(dphase)
                {
                    // EEPROM aktualisieren... 
                    // TG-Zaehler um 1 erhoehen 
                    t1 = eeprom_read_byte((uint8_t*)24) + 256 * eeprom_read_byte((uint8_t*)25) + 1; // Alten Wert holen 
                    cli();
                    while(!eeprom_is_ready());
                    eeprom_write_byte((uint8_t*)24, t1 & 0x00FF);         // LoByte 
                    while(!eeprom_is_ready());
                    eeprom_write_byte((uint8_t*)25, (t1 & 0xFF00) / 256); // HiByte 

                    // Gesamttauchzeit erhoehen 
                    t1 = eeprom_read_byte((uint8_t*)26) + 256 * eeprom_read_byte((uint8_t*)27) + (int)(diveseconds / 60); // Alten Wert holen und veraendern 
                    while(!eeprom_is_ready());
                    eeprom_write_byte((uint8_t*)26, t1 & 0x00FF);           // LoByte 
                    while(!eeprom_is_ready());
                    eeprom_write_byte((uint8_t*)27, (t1 & 0xFF00) / 256);  // HiByte 

                    // Maximaltiefe evtl. erhoehen 
                    if(maxdepth > eeprom_read_byte((uint8_t*)28) + 256 * eeprom_read_byte((uint8_t*)29)) // Alten Wert holen 
                    {
                        while(!eeprom_is_ready());
                        eeprom_write_byte((uint8_t*)28, maxdepth & 0x00FF);         // LoByte 
                        while(!eeprom_is_ready());
                        eeprom_write_byte((uint8_t*)29, (maxdepth & 0xFF00) / 256); // HiByte 
                    }
                    sei();

                    // Indikator fuer Profilende 
                    eeprom_store_byte(230);

                    // Tauchzeit in [min] 
                    eeprom_store_byte((diveseconds / 60) & 0x00FF);          // Lo 
                    eeprom_store_byte(((diveseconds / 60) & 0xFF00) / 256);  // Hi 

                    // Max. Tiefe in [dm] 
                    eeprom_store_byte(maxdepth  & 0x00FF);          // Lo 
                    eeprom_store_byte((maxdepth  & 0xFF00) / 256); // Hi 

                    // Min. Temperatur 
                    eeprom_store_byte(temp_min);

                    // Temperatur auf max. Tauchtiefe 
                    eeprom_store_byte(temp_maxdepth);

                    // Dekostufen 
                    eeprom_store_byte(231);
                    for(t1 = 0; t1 < MAX_DECO_STEPS; t1++)
                        eeprom_store_byte(rcd_decotime[t1]);
                    eeprom_store_byte(232);

                    // Nr. des TG 
                    eeprom_store_byte(eeprom_read_byte((uint8_t*)24) + 256 * eeprom_read_byte((uint8_t*)25));

                    // Tages ZNS 
                    eeprom_store_byte((int)cns_day  & 0x00FF);          // Lo 
                    eeprom_store_byte(((int)cns_day & 0xFF00) / 256);  // Hi 

                    // Tauchgangs-ZNS 
                    eeprom_store_byte((int)cns_dive  & 0x00FF);        // Lo 
                    eeprom_store_byte(((int)cns_dive & 0xFF00) / 256); // Hi 

                    // OTU 
                    eeprom_store_byte((int)otu  & 0x00FF);         // Lo  
                    eeprom_store_byte(((int)otu & 0xFF00) / 256);  // Hi 

                    // Sequenzende 
                    eeprom_store_byte(233);

                    // Speichern der letzten Adresse bei Offset 30 & 31 
                    cli();
                    while(!eeprom_is_ready());
                    eeprom_write_byte((uint8_t*)30, eeprom_byte_count & 0x00FF);            // LoByte 
                    while(!eeprom_is_ready());
                    eeprom_write_byte((uint8_t*)31, (eeprom_byte_count & 0xFF00) / 256);   // HiByte 
                    sei();
                }
                dphase = 0;
            }
            surf_seconds++;
        }

        // Informationen der OFP alle 2 sec. wechseln 
        if(!dphase && !deco_minutes_total) // Bei WT = 0 m, 0 Deco und 5 min. ausgetaucht umschalten 
        {                                   // auf Anzeige der TG-Daten in der Zeile 1 
            if(runseconds > seconds_old3 + 2)
            {
                switch(info_mode)
                {
                  case 0:
                    lcd_linecls(1, 10);
               lcd_putstring(1, 0, "OFP: ");
               surf_hrs = surf_seconds / 3600; // (1/60)²     
               surf_mins = (surf_seconds - surf_hrs * 3600) / 60;
                    xpos = lcd_putnumber(1, 5, surf_hrs, 2, -1, 'l', 1) + 5;
                    lcd_putstring(1, xpos++, ":");
               xpos = lcd_putnumber(1, xpos, surf_mins, 2, -1, 'l', 1) + 5;
                    break;

                  case 1:
                    nft = calc_no_fly_time();
                    if(nft)
                    {
                        lcd_linecls(1, 10);
                        lcd_putstring(1, 0, "FVB: ");
                        xpos = lcd_putnumber(1, 5, nft, -1, -1, 'l', 1) + 5;
                        lcd_putstring(1, xpos, "h");
                    }
                    break;

                  case 2:
                    if(cns_dive)
                    {
                        lcd_linecls(1, 10);
                        lcd_putstring(1, 0, "ZNS TG: ");
                        xpos = lcd_putnumber(1, 8, cns_dive, -1, -1, 'l', 1) + 8;
                        lcd_putstring(1, xpos, "%");
                    }
                    break;

                  case 3:
                    if(cns_day)
                    {
                        lcd_linecls(1, 10);
                        lcd_putstring(1, 0, "ZNS D: ");
                        xpos = lcd_putnumber(1, 7, cns_day, -1, -1, 'l', 1) + 7;
                        lcd_putstring(1, xpos, "%");
                    }
                    break;

                  case 4:
                    if(otu)
                    {
                        lcd_linecls(1, 10);
                        lcd_putstring(1, 0, "OTU: ");
                        xpos = lcd_putnumber(1, 5, otu, -1, -1, 'l', 1) + 5;
                        lcd_putstring(1, xpos, "%");
                    }
                    break;

              case 5: // Dekostufen anzeigen 
                    {
                   is_deco = 0;
                   for(t1 = MAX_DECO_STEPS - 1; t1 >= 0; t1--)
                   {
                     if(rcd_decotime[t1] > 0)
                      is_deco = 1;
                    }

                   if(is_deco)
                   {
                     lcd_linecls(1, 15);
                     lcd_putstring(1, 0, "DEC:");
                     xpos = 5;
                          for(t1 = MAX_DECO_STEPS - 1; t1 >= 0; t1--)
                     {
                       if(rcd_decotime[t1] > 0)
                        xpos += lcd_putnumber(1, xpos, rcd_decotime[t1], -1, -1, 'l', 1) + 1;
                     }
                       }
               }
             }

            if(info_mode == 5)
            {
                lcd_linecls(1, 10);
                show_accu_voltage();
            }

            // Anzeige ppN2 nach TG 
            if(info_mode > 6 && show_ppN2)
            {
                lcd_linecls(1, 15);
               lcd_putstring(1, 0, "ppIg");
               xpos = lcd_putnumber(1, 4, cur_comp + 1, -1, -1, 'l', 1) + 4;
               lcd_putstring(1, xpos, ":");

               lcd_putnumber(1, xpos + 2, piN2[cur_comp++] * 1000, 4, 3, 'l', 1);
               if(cur_comp > 15)
                 cur_comp = 0;
            }

                if(show_ppN2)
                max_info_mode = 22;
            else
                    max_info_mode = 6;

                if(info_mode < max_info_mode)
                    info_mode++;
                else
                    info_mode = 0;

                seconds_old3 = runseconds;
            }
        }

        //  Alle 10 sec. Gewebesaettigung & Dekorechnung 
        if(runseconds >= seconds_old1 + 10)
        {

         if(dphase)
         {
              lcd_cls();
         }

            get_tsensor();

            if(temp <= 8 && !temp_low && dphase)
            {
                temp_low = 1;
                set_ab_values(f_cons + 1, 0);
            }

            // Saettigungsrechnung 
            calc_p_inert_gas(depth * 0.1);
            calc_deco();

            // TG-Profilpunkt speichern als Absolutwert in [m] in 1 Byte alle 20s 
            if(do_record_depth >= 1)
            {
             eeprom_store_byte((unsigned char) (depth * .1));
                do_record_depth = 0;
            }
            else
                do_record_depth++;

            ppo2_exceeded = 0;
            decostep_skipped = 0;
            seconds_old1 = runseconds;
        }

        // Jede Minute ZNS und OTU berechnen 
        if(runseconds > seconds_old2 + 60)
        {
            calc_cns_otu();
            seconds_old2 = runseconds;
        }

        //  Tastaturabfrage ob Einstellungen gesetzt werden sollen 
        switch(get_keys())
        {
          case 1: // Abfrage ob verschiedene Extrafunktionen ausgeführt werden sollen 
          sbtc2pc();
         display_profile();
            clear_flash(1); // TG Profile loeschen ?
         clear_flash(2); // kompletten Flash loeschen ?
         display_rcd();
         display_log();
         break;


          case 2: // Einstellungen 
            settings();
            break;

          case 3:  // Gaswechsel 
            set_curgas();
        }

        // Mikrocontroller fuer den Rest der Sekunde in Energiesparmodus schalten 
        // AD-Wandler aus 
        ADCSRA = 0;
        // In Sleep-Mode gehen 
        set_sleep_mode (SLEEP_MODE_PWR_SAVE);
        sleep_mode();
    }
    return 0;
}






