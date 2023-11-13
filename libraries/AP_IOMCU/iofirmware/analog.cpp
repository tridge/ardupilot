/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
   analog capture for IOMCU. This uses direct register access to avoid
   using up a DMA channel and to minimise latency. We capture a single
   sample at a time
 */

#include "ch.h"
#include "hal.h"
#include "analog.h"

#if HAL_USE_ADC != TRUE
#error "HAL_USE_ADC must be set"
#endif

extern "C" {
    extern void Vector88();
}

#define STM32_ADC1_NUMBER 18
#define STM32_ADC1_HANDLER Vector88

const uint32_t VSERVO_CHANNEL = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN4);
const uint32_t VRSSI_CHANNEL = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN5);

static uint16_t vrssi_val = 0xFFFF;
static uint16_t vservo_val = 0xFFFF;

/*
  initialise ADC capture
 */
void adc_init(void)
{
    rccEnableADC1(true);
    ADC1->CR1 = 0;
    ADC1->CR2 = ADC_CR2_ADON;

    /* Reset calibration just to be safe.*/
    ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_RSTCAL;
    while ((ADC1->CR2 & ADC_CR2_RSTCAL) != 0)
      ;

    /* Calibration.*/
    ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_CAL;
    while ((ADC1->CR2 & ADC_CR2_CAL) != 0)
      ;

    /* set channels 4 and 5 for 28.5us sample time */
    ADC1->SMPR2 = ADC_SMPR2_SMP_AN4(ADC_SAMPLE_28P5) | ADC_SMPR2_SMP_AN5(ADC_SAMPLE_28P5);

    /* capture a single sample at a time */
    ADC1->SQR1 = 0;
    ADC1->SQR2 = 0;

    ADC1->CR1 |= ADC_CR1_EOCIE;

    nvicEnableVector(STM32_ADC1_NUMBER, STM32_ADC_ADC1_IRQ_PRIORITY);
}

/*
  capture one sample on a channel
 */
static void adc_sample_channel(uint32_t channel)
{
    chSysLock();

    if (ADC1->SR & ADC_SR_STRT) {
        return; // still waiting for sample
    }

    /* capture another sample */
    ADC1->SQR3 = channel;
    ADC1->CR2 |= ADC_CR2_ADON;
    chSysUnlock();
}

/*
  capture VSERVO in mV
 */
uint16_t adc_sample_vservo(void)
{
    adc_sample_channel(VSERVO_CHANNEL);
    return vservo_val;
}

/*
  capture VRSSI in mV
 */
uint16_t adc_sample_vrssi(void)
{
    adc_sample_channel(VRSSI_CHANNEL);
    return vrssi_val;
}

static void adc_read_sample()
{
    if (ADC1->SR & ADC_SR_EOC) {
        if (ADC1->SQR3 == VSERVO_CHANNEL) {
            vservo_val = ADC1->DR;
        } else if (ADC1->SQR3 == VRSSI_CHANNEL) {
            vrssi_val = ADC1->DR;
        }
        ADC1->SR &= ~(ADC_SR_EOC | ADC_SR_STRT);
    }
}

OSAL_IRQ_HANDLER(STM32_ADC1_HANDLER) {
    OSAL_IRQ_PROLOGUE();

    chSysLockFromISR();

    adc_read_sample();

    chSysUnlockFromISR();

    OSAL_IRQ_EPILOGUE();
}
