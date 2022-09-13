// Basic RTOS Framework - Spring 2022
// No memory protection, no privilege enforcement
// J Losh

// Student Name:
// TO DO: Add your name on this line.  Do not include your ID number in the file.

// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDs on these pins:
// Blue:   PF2 (on-board)
// Red:    PA2
// Orange: PA3
// Yellow: PA4
// Green:  PE0
// PBs on these pins
// PB0:    PC4
// PB1:    PC5
// PB2:    PC6
// PB3:    PC7
// PB4:    PD6
// PB5:    PD7
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "wait.h"
#include <stdlib.h>
#include <stdio.h>

extern void swpop(uint32_t a);
extern void pspset(uint32_t a);
extern void* swpush();
extern void idle2XPSR();
extern void idel2forPC(uint32_t c);
extern void idel2forR0toLR();
extern uint8_t get_SV_val();
extern uint32_t getR0();

// REQUIRED: correct these bitbanding references for the off-board LEDs and pushbuttons
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) // off-board red LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 0*4))) // off-board green LED
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4))) // off-board yellow LED
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4))) // off-board orange LED

#define PB0 (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4)))
#define PB1 (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))
#define PB2 (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))
#define PB3 (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))
#define PB4 (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 6*4)))
#define PB5 (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 7*4)))
//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*fn)();
// DEBUG
#define DEBUG
//LED and Pushbutton_Mask

#define BLUE_LED_MASK 4
#define RED_LED_MASK 4
#define GREEN_LED_MASK 1
#define YELLOW_LED_MASK 16
#define ORANGE_LED_MASK 8

#define PB0_MASK  16
#define PB1_MASK  32
#define PB2_MASK  64
#define PB3_MASK  128
#define PB4_MASK  64
#define PB5_MASK  128
#define PB_NO_KEY 0

//define stack size
#define stack_size 16
#define MAX_CHARS               80
#define MAX_FIELDS              50
// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 2
typedef struct _semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
    char semaphorename[16];
} semaphore;

semaphore semaphores[MAX_SEMAPHORES];
#define keyPressed 1
#define keyReleased 2
#define flashReq 3
#define resource 4

typedef struct _USER_DATA
{
    char buffer[MAX_CHARS + 1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    uint8_t offset[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;

USER_DATA data;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore
#define STATE_HOLD       5

#define MAX_TASKS 12       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks
uint32_t pidCounter = 0;   // incremented on each thread created
uint16_t s;
uint8_t i = 0;
uint16_t sumcount = 0;
char strm[stack_size];

bool piflag = true;

bool firstcall = true;
bool processflag = false;
bool prempt = true;
bool priorityon = true;

uint32_t *heapBase;
uint32_t sum[MAX_TASKS];

// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    uint32_t pid;                  // PID
    fn pFn;                        // function pointer
    void *spInit;                  // original top of stack
    void *sp;                      // current stack pointer
    int8_t priority;               // 0=highest to 7=lowest
    uint32_t ticks;                // ticks until sleep complete
    char name[16];                 // name of task used in ps command
    uint8_t s;                 // index of semaphore that is blocking the thread
    uint32_t sum0;
    uint32_t sum1;

} tcb[MAX_TASKS];


//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

// REQUIRED: initialize systick for 1ms system timer
void initRtos()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pFn = 0;
    }
}

// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()
{

    if (priorityon)
    {
        uint8_t prioritynum, count, task;
        bool okp, pflag;
        okp = false;
        pflag = false;
        count = taskCurrent;
        task = taskCurrent;
        prioritynum = 0;
        while (!okp)
        {
            if (!firstcall)
            {
                count++;
            }
            firstcall = false;
            if (count == task + 1)
            {
                if (pflag)
                {
                    prioritynum++;
                    if (prioritynum > 8)
                    {
                        priorityon = 0;
                    }

                }
                pflag = true;
            }

            if (count >= MAX_TASKS)
            {
                count = 0;
            }
            if (tcb[count].priority == prioritynum)
            {
                if (tcb[count].state == STATE_READY
                        || tcb[count].state == STATE_UNRUN)
                {
                    okp = true;
                }
            }
        }

        return count;
    }

    else
    {
        bool ok;
        static uint8_t task = 0xFF;
        ok = false;
        while (!ok)
        {
            task++;
            if (task >= MAX_TASKS)
                task = 0;
            ok = (tcb[task].state == STATE_READY
                    || tcb[task].state == STATE_UNRUN);
        }
        return task;
    }
}

void getsUart0(USER_DATA *data)
{
    int8_t o;
    for (o = 0; o <= 80; o++)
    {
        data->buffer[o] = '\0';
    }
    uint32_t count = 0;
    char c;

    getc: ;
    c = getcUart0();
    if (c == 8 || c == 127)
    {
        if (count > 0)
        {

            count--;
            putcUart0(c);
            goto getc;
        }
        else
        {
            goto getc;
        }
    }

    else if (c == 13 || c == 10)
    {
        putcUart0('\0');
        putcUart0('\n');
        putcUart0('\r');
        goto exit;

    }
    else if (c >= 32)
    {
        data->buffer[count] = c;
        count++;
        if (c >= 48 && c <= 57)
        {

        }
        putcUart0(c);
        if (count == MAX_CHARS)
        {
            goto exit;
        }
        else
        {
            goto getc;

        }
    }
    else
    {
        goto getc;
    }
    exit: ;
}

void parseF(USER_DATA *data)
{
    int i;
    data->fieldCount = 0;
    char temp[80];
    temp[0] = 'd';
    for (i = 0; data->buffer[i] != '\0'; i++)
    {
        if ((data->buffer[i] >= 48 && data->buffer[i] <= 57)
                || (data->buffer[i] == 45 || data->buffer[i] == 46))
        {
            data->fieldType[data->fieldCount] = 'n';
        }
        else if ((data->buffer[i] >= 65 && data->buffer[i] <= 90)
                || (data->buffer[i] >= 97 && data->buffer[i] <= 122))
        {
            data->fieldType[data->fieldCount] = 'a';
        }
        else
        {
            temp[i] = 'd';
            data->fieldType[i] = 'd';
            data->buffer[i] = '\0';
        }

        if (temp[i] == 'd'
                && ((data->buffer[i] >= 65 && data->buffer[i] <= 90)
                        || (data->buffer[i] >= 97 && data->buffer[i] <= 122)
                        || (data->buffer[i] >= 48 && data->buffer[i] <= 57)
                        || (data->buffer[i] == 45 || data->buffer[i] == 46)))
        {
            data->fieldPosition[data->fieldCount] = i;
            data->fieldCount++;

        }
        temp[i + 1] = data->fieldType[i];
    }
}

void copy_str(char str1[], const char str2[])
{
    uint8_t i;
    for (i = 0; str2[i] != '\0'; ++i)
    {
        str1[i] = str2[i];
    }

    str1[i] = '\0';
}

bool createThread(fn task, const char name[], uint8_t priority,
                  uint32_t stackBytes)
{
    bool ok = false;
    uint8_t i = 0;
//    uint8_t j = 0;
    bool found = false;
    static uint32_t totalBytes = 0;
    uint32_t Threadlocation = 0;
    // REQUIRED:
    // store the thread name
    // allocate stack space and store top of stack in sp and spInit
    // add task if room in task list

    heapBase = (uint32_t*) (0x20002000);
    totalBytes += stackBytes / 4;
    Threadlocation = (uint32_t) (heapBase + totalBytes);

    if (taskCount < MAX_TASKS)
    {
        // make sure task not already in list (prevent reentrancy)
        while (!found && (i < 12))
        {
            found = (tcb[i++].pFn == task);
        }
        if (!found)
        {

            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID)
            {
                i++;
            }
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = pidCounter++;
            copy_str(tcb[i].name, name);
            tcb[i].pFn = task;
            tcb[i].sp = (void*) (Threadlocation);
            tcb[i].spInit = (void*) (Threadlocation);

#ifdef DEBUG
            //   sprintf(strm, "stack beg for thread =%p\n", tcb[i].sp);
            //   putsUart0(strm);
#endif

            tcb[i].priority = priority;
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    return ok;
}

// REQUIRED: modify this function to restart a thread
void restartThread(fn task)
{
    uint8_t i = 0;

    while (i < 50)
    {
        if (tcb[i].pFn == task && tcb[i].state == STATE_HOLD)
        {
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = pidCounter++;
            tcb[i].sp = tcb[i].spInit;
            tcb[i].ticks=0;
            break;
        }
        i++;
    }

}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
// NOTE: see notes in class for strategies on whether stack is freed or not
void destroyThread(fn task)
{
    int i = 0;
    while (i < MAX_TASKS)
    {
        if (tcb[i].pFn == task)
        {
            tcb[i].state = STATE_HOLD;
            if (i == semaphores[s].processQueue[0]
                    && i == semaphores[s].processQueue[1])
            {
                semaphores[s].processQueue[0] = 0;
                semaphores[s].processQueue[1] = 0;
            }
            if (i == semaphores[s].processQueue[0])
            {

                semaphores[s].processQueue[0] = semaphores[s].processQueue[1];
                semaphores[s].processQueue[1] = 0;
                semaphores[s].queueSize--;
            }
            else if (i == semaphores[s].processQueue[1])
            {
                semaphores[s].queueSize--;

            }
            if (semaphores[3].processQueue[0] == 3)
            {
                semaphores[3].processQueue[0] = 0;
            }

        }
        i++;
    }

}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(fn task, uint8_t priority)
{
    for (i = 0; i < MAX_TASKS; i++)
    {
        if (tcb[i].pFn == task)
        {
            break;
        }
    }

    tcb[i].priority = priority;

}

bool createSemaphore(uint8_t semaphore, uint8_t count)
{
    bool ok = (semaphore < MAX_SEMAPHORES);
    {
        semaphores[semaphore].count = count;
    }
    return ok;
}

// REQUIRED: modify this function to start the operating system
// by calling scheduler, setting PSP, ASP bit, and PC
void startRtos()
{
    taskCurrent = rtosScheduler();
    tcb[taskCurrent].state = STATE_READY;

    pspset((uint32_t) tcb[taskCurrent].sp);

    fn task = tcb[taskCurrent].pFn;

    //timer configurration
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;      // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;    // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = 17;                     // one shot and count up

   // systick configuration
    NVIC_ST_RELOAD_R = 40000;
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_ENABLE;

    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on time
    task();
}

bool isCommand(USER_DATA *data, const char strCommand[], uint8_t minArguments)
{
    uint8_t m = 0;
    uint8_t j = 0;
    uint8_t k = 1;
    bool valid = true;

    while (data->buffer[j] != '\0')
    {
        if (strCommand[m] != data->buffer[j])
        {
            valid = false;
            return (valid);

        }
        m++;
        j++;
    }

    if (strCommand == "ps" || strCommand == "ipcs" || strCommand == "reboot")
    {

        if (data->fieldType[k] == 'a')
        {
            valid = true;
            return (valid);
        }

    }
    else if (strCommand == "kill" || strCommand == "run")
    {

        if (data->fieldType[k] == 'n')
        {
            valid = true;
            return (valid);
        }

    }
    else if (strCommand == "preemption" || strCommand == "scheduler"
            || strCommand == "pidof")
    {

        {
            if (data->fieldType[k] == 'a')
            {
                valid = true;
                return (valid);
            }

        }

    }

    else
    {
        valid = false;
        return (valid);
    }

}

char* getFieldString(USER_DATA *data, uint8_t fieldNumber)
{
    int i = fieldNumber;

    if (i == fieldNumber)
    {
        return &data->buffer[data->fieldPosition[i]];
    }
    else
    {
        return (0);
    }

}

int32_t getFieldInteger(USER_DATA *data, uint8_t fieldNumber)
{
    int num = 0;
    int i;
    if (fieldNumber <= data->fieldCount)
    {
        for (i = data->fieldPosition[fieldNumber]; data->buffer[i] != '\0'; i++)

            num = num * 10 + (data->buffer[i] - 48);
        return num;

    }
    else
    {
        return 0;
    }
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
void yield()
{
    __asm(" SVC #14");

}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
void sleep(uint32_t tick)
{
    __asm(" SVC #20");
}

// REQUIRED: modify this function to wait a semaphore using pendsv
void wait(int8_t s)
{
    __asm(" SVC  #30 ");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(int8_t s)
{
    __asm(" SVC  #40 ");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    uint8_t i;
    sumcount++;
    if (sumcount == 1000)
    {
        sumcount = 0;
        processflag = !processflag;
        if (processflag)
        {
            for (i = 0; i < MAX_TASKS; i++)
            {
                tcb[i].sum1 = 0;
            }
        }
        else
        {
            for (i = 0; i < MAX_TASKS; i++)
            {
                tcb[i].sum0 = 0;
            }
        }

    }
    for (i = 0; i < MAX_TASKS; i++)
    {
        if (tcb[i].state == STATE_DELAYED)
        {
            tcb[i].ticks--;
            if (tcb[i].ticks == 0)
            {
                tcb[i].state = STATE_READY;
            }
        }
    }
    if (prempt)
    {
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
    }
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    char a[12];
    tcb[taskCurrent].sp = swpush();
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    if (processflag)
    {
        tcb[taskCurrent].sum1 = tcb[taskCurrent].sum1 + TIMER1_TAV_R;
    }
    else
    {
        tcb[taskCurrent].sum0 = tcb[taskCurrent].sum0 + TIMER1_TAV_R;
    }
    taskCurrent = rtosScheduler();

//    sprintf(a, "%d", taskCurrent);
//    putsUart0(a);
    TIMER1_TAV_R = 0;
    TIMER1_CTL_R |= TIMER_CTL_TAEN;          // turn-on timer    TIMER1_TAV_R=0;
// __asm(" MOV.W R4, #0x10");
    if (tcb[taskCurrent].state == STATE_READY)
        swpop((uint32_t) tcb[taskCurrent].sp);
    else
    {
        pspset((uint32_t) tcb[taskCurrent].sp);
        idle2XPSR();
        idel2forPC((uint32_t) tcb[taskCurrent].pFn);
        idel2forR0toLR();
        tcb[taskCurrent].state = STATE_READY;
    }
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
//    uint8_t s;
//    struct semaphore *s;
    uint8_t p = (uint8_t) get_SV_val();
    switch (p)
    {
    case 14:
    {
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        break;
    }
    case 20:
    {
        tcb[taskCurrent].state = STATE_DELAYED;
        tcb[taskCurrent].ticks = getR0();
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        break;
    }
    case 30:    //wait
    {
        s = (uint16_t) getR0();
        if (semaphores[s].count > 0)
        {
            semaphores[s].count--;
        }
        else
        {
            tcb[taskCurrent].state = STATE_BLOCKED;
            semaphores[s].processQueue[semaphores[s].queueSize] = taskCurrent;
            semaphores[s].queueSize++;
            tcb[taskCurrent].s = s;
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        }
        break;
    }
    case 40:    //post
    {
        s = (uint16_t) getR0();
        semaphores[s].count++;
        if (semaphores[s].queueSize != 0)
        {
            tcb[semaphores[s].processQueue[0]].state = STATE_READY;
            semaphores[s].queueSize--;
            semaphores[s].processQueue[0] = semaphores[s].processQueue[1];
            semaphores[s].processQueue[1] = 0;
            semaphores[s].count--;
        }
        break;
    }
    case 99:    //get data
    {
        if (processflag)

        {
            for (i = 0; i < MAX_TASKS; i++)
            {
                sum[i] = tcb[i].sum0;
            }
        }
        else
        {
            for (i = 0; i < MAX_TASKS; i++)
            {
                sum[i] = tcb[i].sum1;
            }

        }
        break;
    }
    }

}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           6 pushbuttons
void initHw()
{
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0 | SYSCTL_RCGCGPIO_R2
            | SYSCTL_RCGCGPIO_R3 | SYSCTL_RCGCGPIO_R4 | SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;

    GPIO_PORTC_PUR_R |= PB0_MASK | PB1_MASK | PB2_MASK | PB3_MASK;
    GPIO_PORTF_DIR_R |= BLUE_LED_MASK;
    GPIO_PORTA_DIR_R |= RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK;
    GPIO_PORTE_DIR_R |= GREEN_LED_MASK;
    GPIO_PORTC_DIR_R &= ~(PB0_MASK | PB1_MASK | PB2_MASK | PB3_MASK);

    GPIO_PORTF_DR2R_R |= BLUE_LED_MASK;
    GPIO_PORTA_DR2R_R |= RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK;
    GPIO_PORTE_DR2R_R |= GREEN_LED_MASK;
    GPIO_PORTE_DR2R_R |= PB0_MASK | PB1_MASK | PB2_MASK | PB3_MASK;

    GPIO_PORTF_DEN_R |= BLUE_LED_MASK;
    GPIO_PORTA_DEN_R |= RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK;
    GPIO_PORTE_DEN_R |= GREEN_LED_MASK;
    GPIO_PORTC_DEN_R |= PB0_MASK | PB1_MASK | PB2_MASK | PB3_MASK;

    GPIO_PORTD_LOCK_R |= GPIO_LOCK_KEY;
    GPIO_PORTD_CR_R |= PB5_MASK;
    GPIO_PORTD_PUR_R |= PB4_MASK | PB5_MASK;
    GPIO_PORTD_DIR_R &= ~(PB4_MASK | PB5_MASK);
    GPIO_PORTD_DR2R_R |= PB4_MASK | PB5_MASK;
    GPIO_PORTD_DEN_R |= PB4_MASK | PB5_MASK;

}

uint8_t readPbs()
{
    int8_t value = PB_NO_KEY;
    if (!PB0)
        value = 1;
    if (!PB1)
        value = 2;
    if (!PB2)
        value = 4;
    if (!PB3)
        value = 8;
    if (!PB4)
        value = 16;
    if (!PB5)
        value = 32;
    return value;

}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
    while (true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(2500000);
        ORANGE_LED = 0;
        waitMicrosecond(2500000);

        yield();
    }
}

//void idle2()
//{
//    while (true)
//    {
//        BLUE_LED = 1;
//        waitMicrosecond(2500000);
//        BLUE_LED = 0;
//        waitMicrosecond(2500000);
//        yield();
//
//    }
//}

void flash4Hz()
{
    while (true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot()
{
    while (true)
    {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}

void partOfLengthyFn()
{
// represent some lengthy operation
    waitMicrosecond(990);
// give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while (true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        RED_LED ^= 1;
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while (true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }+2w
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0)
        {
            restartThread(flash4Hz);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 32) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while (true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while (true)
    {
        while (readPbs() == 16)
        {
        }
        yield();
    }
}

void important()
{
    while (true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}

bool strcmp(char str1[], char str2[])
{
    uint8_t c = 0;

    while (str1[c] == str2[c])
    {
        if (str1[c] == '\0' || str2[c] == '\0')
            break;
        c++;
    }

    if (str1[c] == '\0' && str2[c] == '\0')
        return true;
    else
        return false;
}

// REQUIRED: add processing for the shell commands through the UART here
void shell()
{
    uint8_t i;
    char ste, strr, a[12];
    int8_t o;
    for (o = 0; o <= 49; o++)
    {
        data.buffer[o] = '\0';
        data.fieldType[o] = '\0';
    }

    uint32_t sumn[12];
    uint8_t sump[12];
    uint32_t totalsum;
    uint32_t summ[12];
    while (true)
    {
        int8_t o;
        for (o = 0; o <= 80; o++)
        {
            data.buffer[o] = '\0';
        }

        getsUart0(&data);
        parseF(&data);

        if (isCommand(&data, "ps", 0))
        {

            __asm(" SVC  #99 ");
            totalsum = 0;
            for (i = 0; i < MAX_TASKS; i++)
            {
                totalsum = totalsum + sum[i];

            }
            for (i = 0; i < MAX_TASKS; i++)
            {
                summ[i] = ((sum[i] * 100) / totalsum);
                // sumi[i]= (sumi[i] *100)/totalsum;
                sumn[i] = sum[i] * 100;
                sump[i] = sumn[i] % totalsum;

            }
            putsUart0(
                    "\r\nPID\t\t processname\t\t priority\t\t state\t\t\t     %CPU time\r\n\r\n");
            for (i = 0; i < MAX_TASKS; i++)
            {
                if (tcb[i].state == STATE_READY || tcb[i].state == STATE_UNRUN
                        || tcb[i].state == STATE_DELAYED
                        || tcb[i].state == STATE_BLOCKED)
                {
                    sprintf(a, "%d", tcb[i].pid);
                    putsUart0(a);

                    putsUart0("\t\t");
                    putsUart0(tcb[i].name);
                    putsUart0("    \t\t  ");

                    strr = tcb[i].priority + '0';
                    putcUart0(strr);
                    putsUart0("\t\t\t");
                    if (tcb[i].state == STATE_READY)
                    {
                        putsUart0("READY     \t\t\t");
                    }
                    if (tcb[i].state == STATE_UNRUN)
                    {
                        putsUart0("UNRUN      \t\t\t");
                    }
                    if (tcb[i].state == STATE_DELAYED)
                    {
                        putsUart0("SLEEP by ");
//                        strr = tcb[i].ticks + '0';
//                        putcUart0(strr);
                        sprintf(a, "%d", tcb[i].ticks);
                        putsUart0(a);
                        putsUart0("\t\t\t");
                    }
                    if (tcb[i].state == STATE_BLOCKED)
                    {
                        putsUart0("BLOCKED by ");
                        ste = tcb[i].s + '0';
                        putcUart0(ste);
                        putsUart0("\t\t\t");
                    }

//                    strr = (summ[i]) + '0';
//                    putcUart0(strr);
                    sprintf(a, "%d", summ[i]);
                    putsUart0(a);
                    putsUart0(".");
                    sprintf(a, "%d", sump[i]);
                    putsUart0(a);
                    putsUart0("\r\n\r\n");
                }

            }
            putsUart0("\r\n");

        }
        if (isCommand(&data, "reboot", 0))
        {
            putsUart0("\r\n");
            NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
        }

        if (isCommand(&data, "ipcs", 0))
        {

            putsUart0("\r\nsemaphore\t\t waitcount\t    count\r\n\r\n");

            s = keyPressed;

            sprintf(a, "%s", tcb[s].name);
            putsUart0(a);
            putsUart0("\t   ");
            strr = semaphores[s].queueSize + '0';
            putcUart0(strr);

            putsUart0("\t\t      ");
            strr = semaphores[s].count + '0';

            putcUart0(strr);
            putsUart0("\r\n\r\n");

            s = keyReleased;

            sprintf(a, "%s", tcb[s].name);
            putsUart0(a);
            putsUart0("\t   ");
            strr = semaphores[s].queueSize + '0';
            putcUart0(strr);

            putsUart0("\t\t      ");
            strr = semaphores[s].count + '0';

            putcUart0(strr);
            putsUart0("\r\n\r\n");

            s = flashReq;

            sprintf(a, "%s", tcb[s].name);
            putsUart0(a);
            putsUart0("\t\t   ");
            strr = semaphores[s].queueSize + '0';
            putcUart0(strr);

            putsUart0("\t\t      ");
            strr = semaphores[s].count + '0';

            putcUart0(strr);
            putsUart0("\r\n\r\n");

            s = resource;

            sprintf(a, "%s", tcb[s].name);
            putsUart0(a);
            putsUart0("\t   ");
            strr = semaphores[s].queueSize + '0';
            putcUart0(strr);

            putsUart0("\t\t      ");
            strr = semaphores[s].count + '0';

            putcUart0(strr);
            putsUart0("\r\n\r\n");
        }

        if (isCommand(&data, "preemption", 1))
        {

            {
                char *str = getFieldString(&data, 1);

                char alert[] = "on";
                char alert1[] = "off";
                int8_t n = 1, l0 = 0, l1 = 0;
                if (*str == 'o')
                {
                    str++;
                    while (*str != '\0')
                    {
                        if (*(str) == alert[n])
                        {
                            l0++;
                        }
                        else if (*(str) == alert1[n])
                        {
                            l1++;
                        }
                        else
                        {

                        }
                        n++;
                        str++;

                    }
                }

                if (l0 == 1)
                {
                    prempt = true;

                }
                else if (l1 == 2)
                {
                    prempt = false;

                }

            }

        }

        if (isCommand(&data, "scheduler", 1))
        {

            char *str = getFieldString(&data, 1);

            char alert[] = "rr";
            char alert1[] = "prio";
            int8_t n = 0, l0 = 0, l1 = 0;
            if (*str == 'r' || *str == 'p')
            {
                while (*str != '\0')
                {
                    if (*(str) == alert[n])
                    {
                        l0++;
                    }
                    if (*(str) == alert1[n])
                    {
                        l1++;
                    }
                    else
                    {

                    }
                    n++;
                    str++;

                }
            }

            if (l0 == 2)
            {
                priorityon = false;
            }
            else if (l1 == 4)
            {
                priorityon = true;
            }

        }

        if (isCommand(&data, "pi", 1))
        {

            char *str = getFieldString(&data, 1);

            char alert[] = "on";
            char alert1[] = "off";
            int8_t n = 0, l0 = 0, l1 = 0;
            if (*str == 'r' || *str == 'p')
            {
                //                       str++;
                while (*str != '\0')
                {
                    if (*(str) == alert[n])
                    {
                        l0++;
                    }
                    if (*(str) == alert1[n])
                    {
                        l1++;
                    }
                    else
                    {

                    }
                    n++;
                    str++;

                }
            }

            if (l0 == 2)
            {
                piflag = true;

            }
            else if (l1 == 3)
            {
                piflag = false;

            }

        }

        if (isCommand(&data, "kill", 1))
        {
            uint32_t pidvalue;
            int i = 0;
            pidvalue = (getFieldInteger(&data, 1));
            while (i < MAX_TASKS)
            {
                if (tcb[i].pid == pidvalue)
                {
                    destroyThread(tcb[i].pFn);
                }
                i++;
            }

            putsUart0("\r\n");
        }

        if (isCommand(&data, "run", 1))
        {

            for (i = 0; i < MAX_TASKS; i++)
            {
                char *str1;
                str1 = (tcb[i].name);
                if (strcmp(str1, getFieldString(&data, 1)))
                {
                    putsUart0("\r\n");
                    restartThread(tcb[i].pFn);
                    break;

                }
            }

        }

        if (isCommand(&data, "pidof", 1))
        {

            for (i = 0; i < MAX_TASKS; i++)
            {
                char *str1, b;
                str1 = (tcb[i].name);
                if (strcmp(str1, getFieldString(&data, 1)))
                {
                    putsUart0("\r\n");
                    putsUart0(tcb[i].name);
                    putsUart0("\t\t");
                    b = tcb[i].pid + '0';
                    putcUart0(b);
                    putsUart0("\t\t");

                }
            }
            putsUart0("\r\n");
        }

    }

}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;

// Initialize hardware
    initHw();
    initUart0();
    initRtos();

// Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);

// Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);


// Initialize semaphores
    createSemaphore(keyPressed, 1);
    createSemaphore(keyReleased, 0);
    createSemaphore(flashReq, 5);
    createSemaphore(resource, 1);

// Add required idle process at lowest priority
    ok = createThread(idle, "Idle", 7, 1024);
//    ok = createThread(idle2, "Idle2", 7, 1024);
// Add other processes
    ok &= createThread(lengthyFn, "LengthyFn", 6, 1024);
    ok &= createThread(flash4Hz, "Flash4Hz", 4, 1024);
    ok &= createThread(oneshot, "OneShot", 2, 1024);
    ok &= createThread(readKeys, "ReadKeys", 6, 1024);
    ok &= createThread(debounce, "Debounce", 6, 1024);
    ok &= createThread(important, "Important", 0, 1024);
    ok &= createThread(uncooperative, "Uncoop", 6, 1024);
    ok &= createThread(shell, "Shell", 6, 4096);

// Start up RTOS
    if (ok)
        startRtos();    // never returns
    else
        RED_LED = 1;

    return 0;
}
