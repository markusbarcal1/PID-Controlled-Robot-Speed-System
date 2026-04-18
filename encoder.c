#include "encoder.h"
#include "MKL46Z4.h"

/*
 * Encoder pin assignment
 * Left  encoder: A = PTA6,  B = PTA7
 * Right encoder: A = PTA14, B = PTA15
 *
 * interrupt on channel A for each encoder.
 * Channel B is read to get direction.
 */

volatile static int32_t g_leftCount = 0;
volatile static int32_t g_rightCount = 0;

void Encoder_Init(void)
{
    /* Enable clock to PORTA */
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

    /*
     * PTA6  = left encoder A  (interrupt on either edge)
     * PTA7  = left encoder B  (GPIO input)
     * PTA14 = right encoder A (interrupt on either edge)
     * PTA15 = right encoder B (GPIO input)
     *The
     * Pull-ups enabled to keep inputs from floating.
     */
    PORTA->PCR[6]  = PORT_PCR_MUX(1) |
                     PORT_PCR_PE_MASK |
                     PORT_PCR_PS_MASK |
                     PORT_PCR_IRQC(0x0B);

    PORTA->PCR[7]  = PORT_PCR_MUX(1) |
                     PORT_PCR_PE_MASK |
                     PORT_PCR_PS_MASK;

    PORTA->PCR[14] = PORT_PCR_MUX(1) |
                     PORT_PCR_PE_MASK |
                     PORT_PCR_PS_MASK |
                     PORT_PCR_IRQC(0x0B);

    PORTA->PCR[15] = PORT_PCR_MUX(1) |
                     PORT_PCR_PE_MASK |
                     PORT_PCR_PS_MASK;

    /* Set as inputs */
    PTA->PDDR &= ~((1U << 6) | (1U << 7) | (1U << 14) | (1U << 15));

    /* Clear any stale flags before enabling IRQ */
    PORTA->ISFR = (1U << 6) | (1U << 14);

    NVIC_ClearPendingIRQ(PORTA_IRQn);
    NVIC_EnableIRQ(PORTA_IRQn);
}

void Encoder_ResetCounts(void)
{
    __disable_irq();
    g_leftCount = 0;
    g_rightCount = 0;
    __enable_irq();
}

int32_t Encoder_GetLeftCount(void)
{
    return g_leftCount;
}

int32_t Encoder_GetRightCount(void)
{
    return g_rightCount;
}

void PORTA_IRQHandler(void)
{
    uint32_t flags = PORTA->ISFR;

    if (flags & (1U << 6))
    {
        g_leftCount++;
        PORTA->ISFR = (1U << 6);
    }

    if (flags & (1U << 14))
    {
        g_rightCount++;
        PORTA->ISFR = (1U << 14);
    }
}
