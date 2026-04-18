#include "MKL46Z4.h"
#include <stdint.h>
#include <stdbool.h>

#include "motor.h"
#include "encoder.h"

/* ======= Timing ======== */
#define SYSTICK_HZ            1000U   /* 1 ms tick */
#define CONTROL_INTERVAL_MS   10U
#define START_DELAY_MS        2000U
#define RUN_TIME_MS           6000U

/* ====== Baseline motor duties ============ */
#define BASE_DUTY_LEFT        66.0f
#define BASE_DUTY_RIGHT       74.0f

/* ======== Duty limits =============== */
#define DUTY_MIN              45.0f
#define DUTY_MAX              95.0f

/* Speed target
   counts per 10 ms window */
#define TARGET_COUNTS         2.0f

/* ===== PID gains === */
#define LEFT_KP               0.3f
#define LEFT_KI               0.01f
#define LEFT_KD               0.02f

#define RIGHT_KP              0.3f
#define RIGHT_KI              0.01f
#define RIGHT_KD              0.02f

#define INTEGRAL_MIN         -100.0f
#define INTEGRAL_MAX          100.0f

/* ======= State machine ======== */
typedef enum
{
    STATE_IDLE = 0,
    STATE_WAIT_2S,
    STATE_RUN_6S
} robot_state_t;

/* ====== PID structure ====== */
typedef struct
{
    float kp;
    float ki;
    float kd;

    float prevError;
    float integral;
} pid_t;

/* ======= Globals ======== */
volatile robot_state_t g_state = STATE_IDLE;
volatile bool g_startRequest = false;
volatile uint32_t g_stateTimeMs = 0;
volatile uint32_t g_controlDividerMs = 0;

/* encoder snapshots */
volatile int32_t g_prevLeftCount = 0;
volatile int32_t g_prevRightCount = 0;

volatile int32_t g_leftDelta = 0;
volatile int32_t g_rightDelta = 0;
volatile float g_leftDuty = BASE_DUTY_LEFT;
volatile float g_rightDuty = BASE_DUTY_RIGHT;

static pid_t g_leftPID;
static pid_t g_rightPID;

/* ======= Functions ====== */
static void Switch_Init(void);
static void SysTick_Init(void);
static void start_sequence(void);

static void PID_Init(pid_t *pid, float kp, float ki, float kd);
static void PID_Reset(pid_t *pid);
static float PID_Update(pid_t *pid, float setpoint, float measured);

static float clamp_float(float x, float minVal, float maxVal);
static uint8_t duty_to_u8(float duty);

/* ====== Main ====== */
int main(void)
{
    Motor_Init();
    Motor_StopAll();

    Encoder_Init();
    Encoder_ResetCounts();

    Switch_Init();

    PID_Init(&g_leftPID,  LEFT_KP,  LEFT_KI,  LEFT_KD);
    PID_Init(&g_rightPID, RIGHT_KP, RIGHT_KI, RIGHT_KD);

    SystemCoreClockUpdate();
    SysTick_Init();

    while (1)
    {
        if (g_startRequest && g_state == STATE_IDLE)
        {
            g_startRequest = false;
            start_sequence();
        }

        __WFI();
    }
}

/* ====== Init helpers ====== */
static void Switch_Init(void)
{
    /* SW1 on PTC3 */
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

    PORTC->PCR[3] =
        PORT_PCR_MUX(1) |
        PORT_PCR_PE_MASK |
        PORT_PCR_PS_MASK |
        PORT_PCR_IRQC(0x0A);   /* falling edge */

    PTC->PDDR &= ~(1U << 3);

    NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);
    NVIC_EnableIRQ(PORTC_PORTD_IRQn);
}

static void SysTick_Init(void)
{
    SysTick->LOAD = (SystemCoreClock / SYSTICK_HZ) - 1U;
    SysTick->VAL  = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
}

static void start_sequence(void)
{
    __disable_irq();

    g_state = STATE_WAIT_2S;
    g_stateTimeMs = 0;
    g_controlDividerMs = 0;

    Encoder_ResetCounts();
    g_prevLeftCount = 0;
    g_prevRightCount = 0;
    g_leftDelta = 0;
    g_rightDelta = 0;

    g_leftDuty = BASE_DUTY_LEFT;
    g_rightDuty = BASE_DUTY_RIGHT;

    PID_Reset(&g_leftPID);
    PID_Reset(&g_rightPID);

    Motor_StopAll();

    __enable_irq();
}

/* =====  PID helpers ====== */
static void PID_Init(pid_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prevError = 0.0f;
    pid->integral = 0.0f;
}

static void PID_Reset(pid_t *pid)
{
    pid->prevError = 0.0f;
    pid->integral = 0.0f;
}

static float PID_Update(pid_t *pid, float setpoint, float measured)
{
    float error = setpoint - measured;
    float derivative;
    float output;

    pid->integral += error;
    pid->integral = clamp_float(pid->integral, INTEGRAL_MIN, INTEGRAL_MAX);

    derivative = error - pid->prevError;

    output = (pid->kp * error) +
             (pid->ki * pid->integral) +
             (pid->kd * derivative);

    pid->prevError = error;

    return output;
}

/* ===== Utility helpers ======= */
static float clamp_float(float x, float minVal, float maxVal)
{
    if (x < minVal) return minVal;
    if (x > maxVal) return maxVal;
    return x;
}

static uint8_t duty_to_u8(float duty)
{
    duty = clamp_float(duty, 0.0f, 100.0f);
    return (uint8_t)(duty + 0.5f);
}

/* ===== Interrupt handlers ======== */
void PORTC_PORTD_IRQHandler(void)
{
    if (PORTC->ISFR & (1U << 3))
    {
        if (g_state == STATE_IDLE)
        {
            g_startRequest = true;
        }

        PORTC->ISFR = (1U << 3);
    }
}

void SysTick_Handler(void)
{
    switch (g_state)
    {
        case STATE_IDLE:
            break;

        case STATE_WAIT_2S:
            g_stateTimeMs++;

            if (g_stateTimeMs >= START_DELAY_MS)
            {
                g_stateTimeMs = 0;
                g_controlDividerMs = 0;

                g_prevLeftCount = Encoder_GetLeftCount();
                g_prevRightCount = Encoder_GetRightCount();

                g_leftDuty = BASE_DUTY_LEFT;
                g_rightDuty = BASE_DUTY_RIGHT;

                Motor_LeftForward();
                Motor_RightForward();
                Motor_SetBothDutyPercent(duty_to_u8(g_leftDuty),
                                         duty_to_u8(g_rightDuty));

                g_state = STATE_RUN_6S;
            }
            break;

        case STATE_RUN_6S:
        {
            g_stateTimeMs++;
            g_controlDividerMs++;

            if (g_controlDividerMs >= CONTROL_INTERVAL_MS)
            {
                int32_t leftNow;
                int32_t rightNow;
                float leftCorrection;
                float rightCorrection;

                g_controlDividerMs = 0;

                leftNow  = Encoder_GetLeftCount();
                rightNow = Encoder_GetRightCount();

                g_leftDelta  = leftNow  - g_prevLeftCount;
                g_rightDelta = rightNow - g_prevRightCount;

                g_prevLeftCount  = leftNow;
                g_prevRightCount = rightNow;

                leftCorrection  = PID_Update(&g_leftPID,  TARGET_COUNTS, (float)g_leftDelta);
                rightCorrection = PID_Update(&g_rightPID, TARGET_COUNTS, (float)g_rightDelta);

                g_leftDuty  += leftCorrection;
                g_rightDuty += rightCorrection;

                g_leftDuty  = clamp_float(g_leftDuty, DUTY_MIN, DUTY_MAX);
                g_rightDuty = clamp_float(g_rightDuty, DUTY_MIN, DUTY_MAX);

                Motor_SetBothDutyPercent(duty_to_u8(g_leftDuty),
                                         duty_to_u8(g_rightDuty));
            }

            if (g_stateTimeMs >= RUN_TIME_MS)
            {
                Motor_StopAll();
                g_state = STATE_IDLE;
                g_stateTimeMs = 0;
                g_controlDividerMs = 0;
            }
            break;
        }

        default:
            Motor_StopAll();
            g_state = STATE_IDLE;
            g_stateTimeMs = 0;
            g_controlDividerMs = 0;
            break;
    }
}
