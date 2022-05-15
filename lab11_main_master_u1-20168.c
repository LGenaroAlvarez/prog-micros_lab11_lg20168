/*
 * File:   lab11_main-20168.c
 * Author: Luis Genaro Alvarez Sulecio 20168
 * 
 * Programa: Comunicacion master<->slave (full duplex) con envio de datos de un potenciometro
 * leidos en el canal 1 (AN1) enviados a u2 y mostrando valores recibidos de u3 en puerto D
 *
 * Created on May 11, 2022, 5:17 AM
 */
// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF           // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF          // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF          // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF             // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF            // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF          // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF           // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF          // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF            // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V       // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF            // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

//DEFINICION DE FRECUENCIA PARA DELAY
#define _XTAL_FREQ 1000000          // FRECUENCIA PARA DELAYS (1MHz) 

//DEFINICIONES GLOBALES
#define FLAG_SPI 0xFF               // VALOR PARA INICIADO DE RECEPCION DE DATOS

//VARIABLES GLOBALES
uint8_t pot_in = 0;                 // VARIABLE PARA ALMACENAR VALOR DEL POT

//PROTO FUNCIONES
void setup(void);                   // FUNCION DE SETUP

//INTERRUPCIONES
void __interrupt() isr(void){
   
    //---------------------------------MASTER---------------------------------------------
    if (PIR1bits.ADIF){             // REVISAR SI HAY INTERRUPCION DE ADC
        if (ADCON0bits.CHS == 1){   // REVISAR SI HAY INTERRUPCION EN EL CANAL 1
           pot_in = ADRESH;         // CARGAR VALOR DEL ADRESH A VARIABLE DE POTENCIOMETRO
        }
        PIR1bits.ADIF = 0;          // LIMPIAR BANDERA DE INTERRUPCION DE ADC
    }
    //------------------------------------------------------------------------------------
    return;
}

void main(void) {
    //EJECUCION CONFIG
    setup();

    while(1){
        if (ADCON0bits.GO == 0){    // REVISAR SI SE HA INICIADO LA CONVERSION DEL ADC
            ADCON0bits.GO = 1;      // INICIAR PROCESO DE CONVERSION
            __delay_us(40);
        }

        //ACTUALIZACION DEL SLAVE SELECT (SS)
        PORTAbits.RA6 = 1;          // DESHABILITAR ESCLAVO U3 (SS)
        __delay_ms(50);             // TIEMPO DE LECTURA U3
        PORTAbits.RA7 = 0;          // HABILITAR EL ESCLAVO U2 (SS)
        __delay_ms(50);             // TIEMPO DE LECTURA U2

        //ENVIADO DE DATOS AL u2
        SSPBUF = pot_in;            // VALOR DEL POTENCIOMETRO A LA SALIDA DEL MASTER
        while(!SSPSTATbits.BF);     // ESPERAR A QUE SE ESTABILICEN LOS 
        PORTB = pot_in;             // CARGAR VALOR DEL POTENCIOMETRO 

        //ACTUALIZACION DEL SLAVE SELECT (SS)
        PORTAbits.RA7 = 1;          // DESHABILITAR ESCLAVO U2 (SS)
        __delay_ms(50);             // TIEMPO DE LECTURA U2
        PORTAbits.RA6 = 0;          // HABILITAR ESCLAVO U3 (SS)
        __delay_ms(50);             // TIEMPO DE LECTURA U3

        //RECEPCION DE DATOS DEL u2
        SSPBUF = FLAG_SPI;          // ENVIAR VALOR DE ACTIVACION (0xFF)
        while(!SSPSTATbits.BF);     // ESPERAR A QUE SE PROCESE LA TRANSFERENCIA DE DATOS
        PORTD = SSPBUF;             // CARGAR VALOR RECIBIDO A PORTD

        __delay_ms(100);            // TIEMPO DE ESPERA
        
    }
    return;
}

//CONFIGURACION PRINCIPAL
void setup(void){
    ANSEL = 0b00000010;             // AN1 COMO ENTRADA ANALOGICA
    ANSELH = 0;                     // I/O DIGITALES

    TRISA = 0b00000010;             // RA1 COMO ENTRADA
    PORTA = 0;                      // LIMPIEZA DE PORTA

    TRISB = 0;                      // PORTB COMO SALIDA
    PORTB = 0;                      // LIMPIEZA DE PORTB

    TRISD = 0;                      // PORTD COMO SALIDA
    PORTD = 0;                      // LIMPIEZA DE PORTD

    //OSCCONFIC
    OSCCONbits.IRCF = 0b0100;       // FRECUENCIA DE OSCILADOR INTERNO (1MHz)
    OSCCONbits.SCS  = 1;            // RELOJ INTERNO


    //------------------------------MAESTRO-------------------------------------

    TRISC = 0b00010000;             // ENTRADA DE DATOS COMO ENTRADA, SINCRONIZADOR DE RELOJ Y SALIDA DE DATOS COMO SALIDA
    PORTC = 0;                      // LIMPIEZA DE PORTB

    //SSPCON <5:0>
    SSPCONbits.SSPM = 0b0000;       // SPI MAESTRO ACTIVADO | FOSC/4 (250kbits/s)
    SSPCONbits.CKP = 0;             // RELOJ INACTIVO EN 0
    SSPCONbits.SSPEN = 1;           // HABILITACION DE PINES DE SPI
    //SSPSTAT <7:6>
    SSPSTATbits.CKE = 1;            // ENVIO DE DATO EN FLANCO DE SUBIDA
    SSPSTATbits.SMP = 1;            // DATO AL FINAL DE PULSO DE RELOJ
    SSPBUF = 50;                    // VALOR INICIAL ENVIADO AL BUFFER

    //CONFIG DE INTERRUPCIONES
    INTCONbits.GIE = 1;             // HABILITAR INTERRUPCIONES GLOBALES
    INTCONbits.PEIE = 1;            // HABILITAR INTERRUPCIONES EN PERIFERICOS
    PIR1bits.ADIF = 0;              // LIMPIEZA DE BANDERA DE INTERRUPCION DE ADC
    PIE1bits.ADIE = 1;              // HABILITAR INTERRUPCION DE ADC

    //ADC CONFIG
    ADCON0bits.ADCS = 0b01;         // FOSC/8
    ADCON1bits.VCFG0 = 0;           // USO DE VDD COMO VOLTAJE DE REFERENCIA INTERNO
    ADCON1bits.VCFG1 = 0;           // USO DE VSS COMO VOLTAJE DE REFERENCIA INTERNO

    ADCON0bits.CHS = 0b0001;        // SELECCION DE PORTA PIN0 (AN1) COMO ENTRADA DE ADC
    ADCON1bits.ADFM = 0;            // FORMATO DE BITS JUSTIFICADOS A LA IZQUIERDA
    ADCON0bits.ADON = 1;            // HABILITACION DE MODULO DE ADC
    __delay_us(40);                 // TIEMPO DE LECTURA
}