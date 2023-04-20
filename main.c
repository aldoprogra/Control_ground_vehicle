/*
 * Authors: 
 *  Youssef Mohsen Mahmoud Attia 5171925
 *  Giovanni Di Marco            5014077
 *  Sinatra Gesualdo             5159684
 * Readme: 
 *  Please note that the code is designed to receive data in that from  $MCREF,400* from the UART,
 *  if the user should only send the RPM from range to 0 to 1000 RPM, other wise it will be neglected. 
 *  Also, make sure not to flush the UART with characters as the buffer size is set for exactly the indicated
 *  message length with a safety factor of an one or two extra characters.
 *  Please also note that if the user sends a wrong input the LED will stop blinking, ex. $MCREF,1001*
 */
//FOSC
#pragma config FPR = XT                 // Primary Oscillator Mode (XT)
#pragma config FOS = PRI                // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)
//FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)
//FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON          // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI      // Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI      // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN       // PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)
//FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)
//FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

//Libraries included
#include "header.h"
#include "parser.h"

//Global Variables Initialization
typedef struct {
    char *buffer;
    int readIndex;
    int writeIndex;
}CircularBuffer;
typedef struct {
    int n;
    int N;
    void (*f)(void *);
    void* params;
}heartbeat;
typedef struct {		
    float n1;
    float n2;
    float N1;
    float N2;
    float v;
    float w;
    int saturation_flag;
    int timeout_flag;
    int safemode_s5_flag;
    int lcd_toogle_s6_flag;
    int index;
    float array_temp[10];
}data;
data structure;

volatile CircularBuffer RXcircularBuffer;
volatile CircularBuffer TXcircularBuffer;
volatile CircularBuffer TXcircularBuffer_TASK3; // CARLO
static char TX_buffer_size_TASK3[60]; //CARLO
static char TX_buffer_size[20];
static char RX_buffer_size[30];
parser_state pstate;


//Functions Interrupt & Service Routine
void write_buffer(volatile CircularBuffer* cb, char value, int size){
    cb ->buffer[ cb -> writeIndex] = value;
    cb -> writeIndex = (cb -> writeIndex + 1) % size;
    if(cb -> readIndex == cb -> writeIndex){
        cb ->readIndex++;
    }
}
int read_buffer(volatile CircularBuffer *cb, char *value, int size){
    if(cb->readIndex == cb->writeIndex)
        return 0;
    *value = cb->buffer[cb->readIndex];
    cb->readIndex++;
    if(cb->readIndex == size)
        cb->readIndex = 0;
    return 1;
}
int avl_bytes_cb(volatile CircularBuffer* cb, int size){
    if(cb -> readIndex <= cb -> writeIndex) {
        return cb->writeIndex - cb -> readIndex;
    }else{
        return size - cb ->readIndex + cb->writeIndex;
    }
}

void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt(){
    //ISR of UART2
    IFS1bits.U2RXIF = 0;
    char val = U2RXREG;
    write_buffer(&RXcircularBuffer, val, sizeof(RX_buffer_size));
}
void __attribute__((__interrupt__,__auto_psv__)) _U2TXInterrupt(){
    IFS1bits.U2TXIF = 0;
    char packetchar;
    while(U2STAbits.UTXBF == 0) {
        if(read_buffer(&TXcircularBuffer, &packetchar, sizeof(TX_buffer_size)) == 1){
            U2TXREG = packetchar;
        }
        else if(read_buffer(&TXcircularBuffer_TASK3, &packetchar, sizeof(TX_buffer_size_TASK3)) == 1){  // CALRO's   
            U2TXREG = packetchar;
        }
        else{
            break;
        }
    }
}

//Interrupt Service Routine & Functions
void __attribute__((__interrupt__, __auto_psv__)) _INT0Interrupt(){
    //ISR for S5 button
    IFS0bits.INT0IF = 0; //setting the flag down of Interrupt 0 
    
    //Enable safe_mode
    PDC2 = PTPER;
    PDC3 = PTPER;
    structure.safemode_s5_flag = 1;
    structure.n1=structure.N1=structure.n2=structure.N2 = 0;
    
    tmr_setup_period(TIMER2,100); //disable the interrupt for some time for bouncing effect canceling
    IEC0bits.INT0IE = 0; //disable interrupt on S5
    IEC0bits.T2IE = 1; //enable interrupt on timer2
}

void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt(){
    //ISR for S6 button
    IFS1bits.INT1IF = 0; //setting the flag down of Interrupt 1 down
    
    //S6 is pressed, toggle LCD flag
    structure.lcd_toogle_s6_flag = !structure.lcd_toogle_s6_flag; 
    
    tmr_setup_period(TIMER2,100);//disable the interrupt for some time for bouncing effect canceling
    IEC1bits.INT1IE = 0; //disable interrupt on S6
    IEC0bits.T2IE = 1; //enable interrupt on timer2
}

void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt(){
    //Used by s5 & s6 ISRs'
    T2CONbits.TON = 0; //stop the TIMMER2 because its not needed again
    IFS0bits.T2IF = 0; //setting the flag down of timer Interrupt 
    
    IFS0bits.INT0IF = 0; //setting the flag down of Interrupt 0
    IFS1bits.INT1IF = 0; //setting the flag down of Interrupt 1
    IEC0bits.INT0IE = 1; //enable interrupt on s5 again
    IEC1bits.INT1IE = 1; //enable interrupt on s6 again
}
void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(){
    T1CONbits.TON = 0; //stop the TIMMER2 because its not needed again
    IFS0bits.T1IF = 0; //setting the flag down of timer Interrupt
    
    //set flag high.
    structure.timeout_flag = 1;
}

void send_string_uart(char* string){
    int i;
    for(i = 0; string[i] != '\0'; i++) {
        while(U2STAbits.UTXBF == 1);
        U2TXREG = string[i];
    }
}
float get_mean(float *array, int index, float new_element, int size) {  // size = 10
    
    if((index + 1) % size == 1){
        index = 0;}
    else{
            index++;}
    
    array[index] = new_element;


    double sum = 0;
    for (int i = 0; i < size; i++) {
        sum += array[i];
    }

    return sum / size;
}



void task1(void* param) { 
    //1.UART READ ($HLREF,omega,speed* or $HLEWA*)
    //2.set PWM or duty cycle
    //3.read Temp
    //4.LCD print
    data* structure = (data*) param;
    IEC1bits.U2RXIE=0;
    int avl = avl_bytes_cb(&RXcircularBuffer, sizeof(RX_buffer_size));
    IEC1bits.U2RXIE=1;
    int count = 0;
    while (count <avl ){
        char byte;
        IEC1bits.U2RXIE=0;
        read_buffer(&RXcircularBuffer, &byte, sizeof(RX_buffer_size));
        IEC1bits.U2RXIE=1;
        int ret = parse_byte(&pstate, byte);
        if (ret == NEW_MESSAGE){
            if(strcmp(pstate.msg_type, "HLREF") == 0  && structure->safemode_s5_flag ==0){
                if (extract_numbers(pstate.msg_payload, &structure->w, &structure->v)==1){
                    structure->timeout_flag = 0; //reset timeout_flag
                    tmr_setup_period(TIMER1,5000);
                    calculate_wheel_speeds(structure->v,structure->w,&structure->N1, &structure->N2);
                    if (structure->N1 < -limit || structure->N1 > limit || structure->N2 < -limit || structure->N2 > limit) {
                        //Saturation occurred 
                        structure->saturation_flag=1;
                        structure->n1 = (structure->N1 > limit) ? limit : ((structure->N1 < -limit) ? -limit : structure->N1);
                        structure->n2 = (structure->N2 > limit) ? limit : ((structure->N2 < -limit) ? -limit : structure->N2);
                    }
                    else{
                        structure->saturation_flag=0;
                        structure->n1=structure->N1;
                        structure->n2=structure->N2;
                    }
                }
            }
            else if(strcmp(pstate.msg_type, "HLENA" ) == 0){
                //Disable safemode
                structure->safemode_s5_flag = 0;
                structure->n1=structure->N1=structure->n2=structure->N2 = 0;
                //Send ACK of ENA
                char packet[] = "$MCACK,ENA,1*";
                for(int i = 0; packet[i] != '\0'; i++) {
                    write_buffer(&TXcircularBuffer, packet[i], sizeof(TX_buffer_size)); //store packet in TXcircularBuffer
                }
                IEC1bits.U2TXIE=0;
                char packetchar;
                while(U2STAbits.UTXBF==0){
                    if(read_buffer(&TXcircularBuffer, &packetchar, sizeof(TX_buffer_size)) == 1){
                        U2TXREG = packetchar;
                    }
                    else{
                        break;
                    }
                }
                IEC1bits.U2TXIE=1;
            }
        }
        count++;
    }
    //PWM
    double duty_cycle_n1= map(structure->n1,-60,60,0,1); // should we add an offset ?!
    double duty_cycle_n2= map(structure->n2,-60,60,0,1); // should we add an offset ?!
    PDC2 = duty_cycle_n1 * 2 * PTPER;
    PDC3 = duty_cycle_n2 * 2 * PTPER;
    //TEMP part here
    //LCD
    char status = 'C';
    if (structure ->safemode_s5_flag ==1){
        status = 'H';
    }
    else if (structure ->safemode_s5_flag ==0 && structure->timeout_flag == 1){
        status = 'T';
    }
    else{
        status = 'C';
    }
    char row[17];
    sprintf(row, "STATUS: %c",status);
    spi_clear_first_row();
    spi_move_cursor(FIRST_ROW,0);
    spi_put_string(row);
    if (structure ->lcd_toogle_s6_flag == 0){
        sprintf(row, "R:%.1f;%.1f",structure->n1,structure->n2);
        spi_clear_second_row();
        spi_move_cursor(SECOND_ROW,0);
        spi_put_string(row);
    }
    else{
        sprintf(row, "S:%.1f;%.1f",structure->w,structure->v);
        spi_clear_second_row();
        spi_move_cursor(SECOND_ROW,0);
        spi_put_string(row);
    }
}
void task2(void* param) {
    //ASK PROFESSOR WHAT IF ITS BOTH INN SAFEMODE AND TIMEOUT MODE ???!!!
    data* structure = (data*) param;
    int status = 0;
    if(structure ->timeout_flag ==0){
        LATBbits.LATB1=0;
    }
    else if (structure ->timeout_flag ==1){
        LATBbits.LATB1 = !LATBbits.LATB1;
        status = 1;
    }
    if (structure ->safemode_s5_flag ==1 && structure ->timeout_flag ==0){
        status = 2;
    }
    else if(structure ->safemode_s5_flag ==1 && structure ->timeout_flag ==1){
        status = 0;
    }
    
    char packet[22];
    sprintf (packet,"$MCFBK,%.1f,%.1f,%d*", structure->n1, structure->n2,status);
    for(int i = 0; packet[i] != '\0'; i++) {
        write_buffer(&TXcircularBuffer, packet[i], sizeof(TX_buffer_size)); //store packet in TXcircularBuffer
    }
    IEC1bits.U2TXIE=0;
    char packetchar;
    while(U2STAbits.UTXBF==0){
        if(read_buffer(&TXcircularBuffer, &packetchar, sizeof(TX_buffer_size)) == 1){
            U2TXREG = packetchar;
        }
        else{
            break;
        }
    }
    IEC1bits.U2TXIE=1;
}
void task3(void* param) {
    LATBbits.LATB0 = !LATBbits.LATB0;
    send_string_uart("|");
  
    data* structure = (data*) param;
    
    char MCALE[12]; 
    char MCTEM[12]; 

    
    // Increment Timeout to 1 sec more
    structure -> timeout_flag++;
    
    // for debugging
    structure ->saturation_flag = 1;
    structure->N1 = 1.0;
    structure->N2 = 1.0;
    
    
    if(structure ->saturation_flag == 1){ // If velocities outside threshold
        
        // $MCALE,n1,n2*
   
        sprintf(MCALE,"$MCALE,%.1f,%.1f*",structure->N1, structure->N2);
        for(int i = 0; MCALE[i] != '\0'; i++) {
            IEC1bits.U2TXIE=0;
            write_buffer(&TXcircularBuffer_TASK3, MCALE[i], sizeof(TX_buffer_size_TASK3)); 
            IEC1bits.U2TXIE=1;
        }}

    
    send_string_uart("|");
    
    // Temperature aka $MCTEM,temp* 

    ADCON1bits.SAMP = 1; //start sampling
    while(ADCON1bits.DONE == 0); //wait until conversion is done
    ADCON1bits.DONE = 0; //just for double checking
    int tempBits = ADCBUF1; // extract the value from the buffer 
    float temperature;
    temperature = ((tempBits * 5.0/1024.0)-0.75)*100.0+25.0; // Temperature
    float temp_mean;
    temp_mean = get_mean(structure->array_temp,structure->index, temperature, 10);
    
    // $MCTEM,temp* 
    sprintf(MCTEM,"$MCTEM,%.1f*", temp_mean);
    for(int i = 0; MCTEM[i] != '\0'; i++) {
            IEC1bits.U2TXIE = 0;
            write_buffer(&TXcircularBuffer_TASK3, MCTEM[i], sizeof(TX_buffer_size_TASK3)); //store packet in TXcircularBuffer
            IEC1bits.U2TXIE = 1;
        }
    

}
void scheduler(heartbeat schedInfo[]) {
    int i;
    for (i = 0; i < MAX_TASKS; i++) {
        schedInfo[i].n++;
        if (schedInfo[i].n >= schedInfo[i].N) {
            schedInfo[i].f(schedInfo[i].params);
            schedInfo[i].n = 0;
        }
    }
}

void setup(){
    /*I/O SETUP*/
    TRISBbits.TRISB1 = 0; //D4
    /*Interrupt SETUP*/
    IEC0bits.INT0IE = 1; //Interrupt flag HIGH; to enable interrupt on pin S5 button
    IEC1bits.INT1IE = 1; //Interrupt flag HIGH; to enable interrupt on pin S6 button
    IEC0bits.T1IE = 1; //enable interrupt on timer1
    /*PINS setup*/
    /*UART setup*/
    TXcircularBuffer.buffer= TX_buffer_size;
    RXcircularBuffer.buffer= RX_buffer_size;
    TXcircularBuffer_TASK3.buffer = TX_buffer_size_TASK3; // CARLO
    TXcircularBuffer_TASK3.writeIndex = 0; //CARLO
    TXcircularBuffer_TASK3.readIndex = 0; // CARLO
    UART_config();
    
    /*LCD setup*/
    spi_config();
    /*ADC setup*/
    /*PWM setup*/
    pwm_config();
    /*Parser setup*/
    pstate.state = STATE_DOLLAR;
	pstate.index_type = 0; 
	pstate.index_payload = 0;
    
    /*Wait for 1 second*/
    tmr_wait_ms(TIMER4, 1000);
}
int main(void) {
    setup();
    /*scheduler setup*/
    heartbeat schedInfo[MAX_TASKS];
    
    structure.N1 = 0;
    structure.N2 = 0;
    structure.n1 = 0;
    structure.n2 = 0;
    structure.v  = 0;
    structure.w  = 0;
    structure.saturation_flag  = 0;
    structure.timeout_flag  = 0;
    structure.safemode_s5_flag = 0;
    structure.lcd_toogle_s6_flag = 0;
    structure.index = -1;
    //Setting the time of each task
    schedInfo[0].n = 0; 
    schedInfo[0].N = 20; //10hz = 0.1 s = 0.005 * 20
    schedInfo[0].f = task1; 
    schedInfo[0].params = (void*)(&structure);
    schedInfo[1].n = 0;
    schedInfo[1].N = 40;
    schedInfo[1].f = task2;
    schedInfo[1].params = (void*)(&structure); 
    schedInfo[2].n = 0;
    schedInfo[2].N = 200;
    schedInfo[2].f = task3;
    schedInfo[2].params = (void*)(&structure);
    
    
    tmr_setup_period(TIMER3, 5);
    while (1) {
        scheduler(schedInfo);
        tmr_wait_period(TIMER3);
    }
    return 0;
}