;-------------------------------------------------------------------------------
;Encabezado
;-------------------------------------------------------------------------------
    
; Archivo: Prelab_main.s
; Dispositivo: PIC16F887
; Autor: José Fernando de León González
; Compilador:  pic-as (v2.30), MPLABX v5.40
;
; Programa: Contador binario de 4 bits utilizando el TIMER0 y contador hexade-
;	    cimal utilizando una led de 7 segmentos
; Hardware: LEDs  & resistencias en el puerto A (RA0, RA1, RA2, RA3), botones
;	    en el puerto B (RB0, RB1) & display de 7 segmentos en el puerto C
;
; Creado: 7/02/22
; Última modificación: 7/02/22
    
PROCESSOR 16F887

;-------------------------------------------------------------------------------
;Palabras de configuración 
;-------------------------------------------------------------------------------
    
; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
  CONFIG  PWRTE = ON            ; Power-up Timer Enable bit (PWRT enabled)
  CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
  CONFIG  BOREN = OFF            ; Brown Out Reset Selection bits (BOR enabled)
  CONFIG  IESO = OFF             ; Internal External Switchover bit (Internal/External Switchover mode is enabled)
  CONFIG  FCMEN = OFF            ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
  CONFIG  LVP = ON              ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)

;-------------------------------------------------------------------------------
;Librerías incluidas
;-------------------------------------------------------------------------------
  
#include <xc.inc>
 
;-------------------------------------------------------------------------------
;Variables
;-------------------------------------------------------------------------------
  
PSECT udata_shr ; Variables en la memoria RAM compartida entre bancos
    counter: DS 1 ;1 byte reservado (variable para aumentar la cantidad de ciclos del TIMER0)
    
  
;-------------------------------------------------------------------------------
;Vector Reset
;-------------------------------------------------------------------------------

PSECT VectorReset, class = CODE, abs, delta = 2 ; delta = 2: Las instrucciones necesitan 2 localidades para ejecutarse & abs = absolute: indicamos que contamos a partir de 0x0000
ORG 0x0000  ; la localidad del vector reset es 0x0000
 
VectorReset:
    PAGESEL main
    GOTO main

;-------------------------------------------------------------------------------
;Tabla para display de siete segmentos
;-------------------------------------------------------------------------------

PSECT table, class = CODE, abs, delta = 2
ORG 100h 

table:
    CLRF PCLATH
    BSF PCLATH, 0           ; PCLATH en 01
    ANDLW 0X0F
    ADDWF PCL               ; PC = PCLATH + PCL | Sumamos W al PCL para seleccionar un dato en la tabla
    retlw 00111111B         ; 0
    retlw 00000110B         ; 1
    retlw 01011011B         ; 2
    retlw 01001111B         ; 3
    retlw 01100110B         ; 4
    retlw 01101101B         ; 5
    retlw 01111101B         ; 6
    retlw 00000111B         ; 7
    retlw 01111111B         ; 8 
    retlw 01101111B         ; 9
    retlw 01110111B         ; A
    retlw 01111100B         ; b
    retlw 00111001B         ; C
    retlw 01011110B         ; D
    retlw 01111001B         ; C
    retlw 01110001B         ; F
    
;-------------------------------------------------------------------------------
;main (configuración)
;-------------------------------------------------------------------------------

 main:			    
    call config_ports
    call config_clock
    call config_tmr0
    MOVLW 10
    MOVWF counter
    MOVLW 11110000B
    MOVWF PORTD
    banksel PORTA
			    ;---------------------------------------------------
			    ;TEMPORIZACIÓN TIMER0: 100 ms = 4*4us*(256-60)*32
			    ;---------------------------------------------------
			    
;-------------------------------------------------------------------------------
;Loop
;-------------------------------------------------------------------------------

loop:			
    call timer0_counter
    
    
    BTFSC  RB0		    ; Revismos si el bit 0 de PORTB está en 0
    call inc_portD	    ; NO: está presionado, llamar a incrementar D
    BTFSC RB1		    ; SÍ: no está presionado, revisar si el bit 1 de PORTB está en 0
    call dec_portD	    ; NO: está presionado, llamar a decrementar D
    call push_limit_check   ; Esta subrutina revisará los límites impuestos para los contadores y los valores del display en los puertos
    
    
    MOVF PORTD, W	    ; llevamos el valor de PORTD a W      
    call table
    MOVWF PORTC		     
    
    call display_reach_reset ;Esta subrutina reiniciará el contador si su valor llega a ser igual que el del display
    
    goto loop
    
;-------------------------------------------------------------------------------
;subrutinas
;-------------------------------------------------------------------------------

config_clock:
    banksel OSCCON      ;banco 01
    BCF IRCF2
    BSF IRCF1
    BCF IRCF0           ; IRCF <2:0> -> 010 250 kHz
    
    BSF SCS             ;reloj interno
    
    return
    
config_ports:
    banksel ANSEL       ; banco 11
    CLRF ANSEL		; pines digitales
    CLRF ANSELH
    
    banksel TRISA       ; banco 01
    CLRF TRISA		; PORTA como salida
    CLRF TRISD		; PORTD como salida
    CLRF TRISC          ; PORTC como salida
    
    BSF  TRISB0
    BSF  TRISB1         ; pines 1 & 2 del puerto B como entradas
    
    banksel PORTA       ; banco 00
    CLRF PORTA		; limpiamos PORTA
    CLRF PORTD		; limpiamos PORTD
    CLRF PORTC          ; limpiamos PORTC
    return

config_tmr0:
    banksel OPTION_REG  ; banco 01
    BCF T0CS            ; TIMER0 como temporizador
    BCF PSA             ; Prescaler a TIMER0
    BSF PS2
    BCF PS1
    BCF PS0             ; PS<2:0> -> preescaler 100 1:32
    
    call restart_tmr0
    
    return

restart_tmr0:    
    BANKSEL TMR0        ; banco 00
    MOVLW 60            ; cargar valor inicial a W
    MOVWF TMR0          ; cargar el valor inicial al TIMER0
    bcf T0IF            ; limpiar la bandera  de overflow del TIMER0
    
    return

timer0_counter:
    BTFSS T0IF		  ; Chequeamos si la bandera del timer se ha activado
    goto $-1		  ; NO: seguimos esperando
    call restart_tmr0     ; SÍ: llamamos a reiniciar el TIMER0
    DECFSZ counter        ; decrementamos el contador y verificamos si se vuelve 0
    goto $+6	          ;NO: salimos de la subrutina
    call restart_counter  ;SÍ: reiniciamos el contador y seguimos la subrutina
    
    
    INCF PORTA
    BTFSS PORTA, 0
    goto $+1
    BCF PORTD, 5
    return

inc_portD:
    BTFSC RB0		    ; Revisamos si el pin 0 del puerto B está en 0
    goto $-1		    ; NO: el botón sigue presionado, regresar al chequeo
    INCF PORTD		    ; SÍ: el botón dejó de presionarse, Incrementar PORTD

    return

dec_portD:
    BTFSC RB1		    ;Revisamos si el pin 1 del puerto B está en 0
    goto $-1		    ;NO: el botón sigue presionado, regresar al chequeo
    DECF PORTD		    ;SÍ: el botón dejó de presionarse, decrementar PORTD
    
    return

restart_counter:
    MOVLW 10
    MOVWF counter
    
    return

push_limit_check:
    BTFSS PORTA, 4
    goto $+2
    CLRF PORTA
    
    
    BTFSS PORTD, 4	    ;chequeamos si el quinto bit de PORTD está en 1
    goto $+11                ;NO: salimos de la subrutina
    BTFSS PORTD, 7	    ;SÍ, chequeamos si el octavo bit de PORTD está en 1
    goto $+2		    ;NO: Limpiamos el puerto 
    goto $+3                ;SÍ: Limpiamos el puerto y seteamos el nibble low
    CLRF PORTD		    
    goto $+6 
    CLRF PORTD		    
    BSF PORTD, 0
    BSF PORTD, 1
    BSF PORTD, 2
    BSF PORTD, 3
   
    
    return
    
display_reach_reset:
    
    MOVF PORTD, W	    ; Enviamos el valor del PORTD a W
    SUBWF PORTA, W	    ; Restamos el valor de W a PORTA, lo almacenamos en W
    
    BTFSC STATUS, 2	    ; Tras la resta, revisamos si la bandera Z de STATUS es 0
    goto $+2		    ; NO: reiniciamos la cuenta
    goto $+5		    ; SÍ: salimos de la subrutina
    CLRF PORTA		    ; reiniciamos PORTA
    call restart_tmr0	    ; reiniciamos la cuenta del tmr0
    call restart_counter    ; reiniciamos el valor inicial del counter
    
    BSF PORTD, 5
    return
    
END