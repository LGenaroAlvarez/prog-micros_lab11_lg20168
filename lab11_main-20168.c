/*
 * File:   lab11_main-20168.c
 * Author: Luis Genaro Alvarez Sulecio 20168
 * 
 * Programa: Comunicacion master->slave con envio de datos de un potenciometro
 * leidos en el canal 1 (AN1), utilizando RA0 como selector de modo (maestro/esclavo)
 * y mostrando valores recibidos en puerto D
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

//VARIABLES GLOBALES
uint8_t pot_in;                     // VARIABLE PARA ALMACENAR VALOR DEL POT

//PROTO FUNCIONES
void setup(void);                   // FUNCION DE SETUP

//INTERRUPCIONES
void __interrupt() isr(void){
    if (PIR1bits.SSPIF){            // REVISAR INTERRUPCION DE RECEPCION DE DATOS
        __delay_us(40);             // DELAY DE ESPERA PARA ACTUALIZAR DATOS
        PORTD = SSPBUF;             // CARGAR VALORES RECIBIDOS AL PORTD
        PIR1bits.SSPIF = 0;         // LIMPIEZA DE BANDERA DE INTERRUPCION DE SPI
    }
    if (PIR1bits.ADIF){             // REVISAR SI HAY INTERRUPCION DE ADC
        if (ADCON0bits.CHS == 1){   // REVISAR SI HAY INTERRUPCION EN EL CANAL 1
           pot_in = ADRESH;         // CARGAR VALOR DEL ADRESH A VARIABLE DE POTENCIOMETRO
        }
        PIR1bits.ADIF = 0;          // LIMPIAR BANDERA DE INTERRUPCION DE ADC
    }
    return;
}

void main(void) {
    //EJECUCION CONFIG
    setup();
    
    while(1){
        if (ADCON0bits.GO == 0){    // REVISAR SI SE HA INICIADO LA CONVERSION DEL ADC
            ADCON0bits.GO = 1;      // INICIAR PROCESO DE CONVERSION
        }
        if(PORTAbits.RA0){          // SI EN MODO MAESTRO
            __delay_us(40);         // DELAY DE ESPERA PARA ACTUALIZAR DATOS
            if(SSPSTATbits.BF){     // REVISION DE COMUNICACIONES ACTIVAS
                SSPBUF = pot_in;    // SI ESTA LIBRE, CARGAR VALOR DEL POT AL BUFFER PARA ENVIARLO
                PORTD = pot_in;     // CARGAR VALOR AL PORTD PARA CONFIRMAR SINCRONIZACION
            }
        }
    }
    return;
}

//CONFIGURACION PRINCIPAL
void setup(void){
    ANSEL = 0b00000010;             // AN1 COMO ENTRADA ANALOGICA
    ANSELH = 0;                     // I/O DIGITALES
    
    TRISA = 0b00100011;             // RA0, RA1 Y RA5 COMO ENTRADAS
    PORTA = 0;                      // LIMPIEZA DE PORTA
    
    TRISD = 0;                      // PORTD COMO SALIDA
    PORTD = 0;                      // LIMPIEZA DE PORTD
    
    //OSCCONFIC
    OSCCONbits.IRCF = 0b0100;       // FRECUENCIA DE OSCILADOR INTERNO (1MHz)
    OSCCONbits.SCS  = 1;            // RELOJ INTERNO
    
    
    // SI RA0 = 0, PIC EN MODO MAESTRO
    if (PORTAbits.RA0){             
        TRISC = 0b00010000;         // ENTRADA DE DATOS COMO ENTRADA, SINCRONIZADOR DE RELOJ Y SALIDA DE DATOS COMO SALIDA
        PORTC = 0;                  // LIMPIEZA DE PORTB
        
        //SSPCON <5:0>
        SSPCONbits.SSPM = 0b0000;   // SPI MAESTRO ACTIVADO | FOSC/4 (250kbits/s)
        SSPCONbits.CKP = 0;         // RELOJ INACTIVO EN 0
        SSPCONbits.SSPEN = 1;       // HABILITACION DE PINES DE SPI
        //SSPSTAT <7:6>             
        SSPSTATbits.CKE = 1;        // ENVIO DE DATO EN FLANCO DE SUBIDA 
        SSPSTATbits.SMP = 1;        // DATO AL FINAL DE PULSO DE RELOJ 
        SSPBUF = pot_in;            // VALOR INICIAL ENVIADO AL BUFFER
        
        //CONFIG DE INTERRUPCIONES
        INTCONbits.GIE = 1;         // HABILITAR INTERRUPCIONES GLOBALES
        INTCONbits.PEIE = 1;        // HABILITAR INTERRUPCIONES EN PERIFERICOS
        PIR1bits.ADIF = 0;          // LIMPIEZA DE BANDERA DE INTERRUPCION DE ADC
        PIE1bits.ADIE = 1;          // HABILITAR INTERRUPCION DE ADC
        
        //ADC CONFIG
        ADCON0bits.ADCS = 0b01;     // FOSC/8
        ADCON1bits.VCFG0 = 0;       // USO DE VDD COMO VOLTAJE DE REFERENCIA INTERNO
        ADCON1bits.VCFG1 = 0;       // USO DE VSS COMO VOLTAJE DE REFERENCIA INTERNO

        ADCON0bits.CHS = 0b0001;    // SELECCION DE PORTA PIN0 (AN1) COMO ENTRADA DE ADC
        ADCON1bits.ADFM = 0;        // FORMATO DE BITS JUSTIFICADOS A LA IZQUIERDA
        ADCON0bits.ADON = 1;        // HABILITACION DE MODULO DE ADC
        __delay_us(40);             // TIEMPO DE LECTURA
    }
    
    // EN CASO DE RA0 = 0, PIC EN MODO ESCLAVO
    else{                           
        TRISC = 0b00011000;         // ENTRADA DE DATOS Y SINCRONIZADOR DE RELOJ COMO ENTRADA, SALIDA DE DATOS COMO SALIDA
        PORTC = 0;                  // LIMPIEZA DE PORTD
        
        //SSPCON <5:0>
        SSPCONbits.SSPM = 0b0100;   // SS HABILITADO, SPI EN MODO ESCLAVO
        SSPCONbits.CKP = 0;         // RELOJ INACTIVO EN 0
        SSPCONbits.SSPEN = 1;       // HABILITACION DE PINES DE SPI
        //SSPSTAT <7:6>
        SSPSTATbits.CKE = 1;        // ENVIO DE DATO EN FLANCO DE SUBIDA 
        SSPSTATbits.SMP = 0;        // DATO AL FINAL DE PULSO DE RELOJ (EN 0 DEBIDO A MODO ESCLAVO)
        
        //CONFIG DE INTERRUPCIONES
        PIR1bits.SSPIF = 0;         // LIMPIAR BANDERA DE INTERRUPCIONES DE SP1
        PIE1bits.SSPIE = 1;         // ACTIVAR INTERRUPCIONES DE SPI
        INTCONbits.GIE = 1;         // ACTIVAR INTERRUPCIONES GLOBALES
        INTCONbits.PEIE = 1;        // ACTIVAR INTERRUPCIONES DE PERIFERICOS
    }   
}