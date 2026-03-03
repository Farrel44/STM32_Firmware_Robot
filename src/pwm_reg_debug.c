/*
 * pwm_reg_debug.c — Register-level PWM truth verification.
 *
 * HOW TO USE:
 *   In main.c, AFTER MX_TIM1_Init() / MX_TIM2_Init() / MX_GPIO_Init()
 *   and AFTER the boot self-test block, call:
 *
 *       extern void PWM_RegDebug_DumpAll(void);
 *       extern void PWM_RegDebug_DriveAndDump(void);
 *
 *       PWM_RegDebug_DumpAll();      // dump static register state
 *       PWM_RegDebug_DriveAndDump(); // drive 40% duty + dump live CCR
 *
 *   Then COMMENT OUT osKernelStart() to keep RTOS from interfering.
 *   Connect a terminal at 115200 baud on /dev/ttyACM0 and read output.
 *
 * This module trusts NOTHING except peripheral registers.
 */

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* Low-level UART print (blocking, no RTOS, no printf dependency)     */
/* ------------------------------------------------------------------ */
static void dbg_print(const char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

static void dbg_hex32(const char *label, uint32_t val)
{
  char buf[64];
  /* Manual hex format to avoid printf dependency issues */
  static const char hex[] = "0123456789ABCDEF";
  int i = 0;
  while (label[i] && i < 40) { buf[i] = label[i]; i++; }
  buf[i++] = '0'; buf[i++] = 'x';
  for (int b = 28; b >= 0; b -= 4) {
    buf[i++] = hex[(val >> b) & 0xF];
  }
  buf[i++] = '\r';  buf[i++] = '\n';  buf[i] = '\0';
  dbg_print(buf);
}

static void dbg_dec(const char *label, uint32_t val)
{
  char buf[64];
  int i = 0;
  while (label[i] && i < 40) { buf[i] = label[i]; i++; }

  /* Decimal conversion */
  char tmp[12];
  int j = 0;
  if (val == 0) { tmp[j++] = '0'; }
  else {
    uint32_t v = val;
    while (v > 0) { tmp[j++] = '0' + (v % 10); v /= 10; }
  }
  for (int k = j - 1; k >= 0; k--) { buf[i++] = tmp[k]; }
  buf[i++] = '\r';  buf[i++] = '\n';  buf[i] = '\0';
  dbg_print(buf);
}

/* ------------------------------------------------------------------ */
/* 1. RCC Clock Enable Verification                                   */
/* ------------------------------------------------------------------ */
static void dump_rcc(void)
{
  dbg_print("\r\n=== RCC CLOCK ENABLE ===\r\n");

  /* TIM1 is on APB2 */
  uint32_t apb2 = RCC->APB2ENR;
  dbg_hex32("RCC->APB2ENR      = ", apb2);
  dbg_print(  (apb2 & RCC_APB2ENR_TIM1EN) ? "  TIM1EN: SET (ok)\r\n"
                                            : "  TIM1EN: CLEAR *** FAULT ***\r\n");

  /* TIM2 is on APB1 */
  uint32_t apb1 = RCC->APB1ENR;
  dbg_hex32("RCC->APB1ENR      = ", apb1);
  dbg_print(  (apb1 & RCC_APB1ENR_TIM2EN) ? "  TIM2EN: SET (ok)\r\n"
                                            : "  TIM2EN: CLEAR *** FAULT ***\r\n");
}

/* ------------------------------------------------------------------ */
/* 2. Timer Register Dump                                             */
/* ------------------------------------------------------------------ */
static void dump_tim_regs(const char *name, TIM_TypeDef *tim, int is_advanced)
{
  dbg_print("\r\n=== ");
  dbg_print(name);
  dbg_print(" REGISTERS ===\r\n");

  dbg_hex32("  CR1  = ", tim->CR1);
  dbg_print(  (tim->CR1 & TIM_CR1_CEN) ? "    CEN: SET (counter running)\r\n"
                                         : "    CEN: CLEAR *** TIMER NOT RUNNING ***\r\n");
  dbg_print(  (tim->CR1 & TIM_CR1_ARPE) ? "    ARPE: SET (preload enabled)\r\n"
                                          : "    ARPE: CLEAR\r\n");

  dbg_hex32("  CR2  = ", tim->CR2);
  dbg_dec(  "  PSC  = ", tim->PSC);
  dbg_dec(  "  ARR  = ", tim->ARR);
  dbg_dec(  "  CNT  = ", tim->CNT);

  dbg_hex32("  CCMR1= ", tim->CCMR1);
  dbg_hex32("  CCMR2= ", tim->CCMR2);
  dbg_hex32("  CCER = ", tim->CCER);

  /* Decode CCER channel enables */
  dbg_print(  (tim->CCER & TIM_CCER_CC1E) ? "    CC1E: ENABLED\r\n"  : "    CC1E: disabled\r\n");
  dbg_print(  (tim->CCER & TIM_CCER_CC2E) ? "    CC2E: ENABLED\r\n"  : "    CC2E: disabled\r\n");
  dbg_print(  (tim->CCER & TIM_CCER_CC3E) ? "    CC3E: ENABLED\r\n"  : "    CC3E: disabled\r\n");
  dbg_print(  (tim->CCER & TIM_CCER_CC4E) ? "    CC4E: ENABLED\r\n"  : "    CC4E: disabled\r\n");

  /* Check polarity (CC1P, etc.) */
  dbg_print(  (tim->CCER & TIM_CCER_CC1P) ? "    CC1P: ACTIVE LOW\r\n"  : "    CC1P: ACTIVE HIGH\r\n");
  dbg_print(  (tim->CCER & TIM_CCER_CC2P) ? "    CC2P: ACTIVE LOW\r\n"  : "    CC2P: ACTIVE HIGH\r\n");

  dbg_dec(  "  CCR1 = ", tim->CCR1);
  dbg_dec(  "  CCR2 = ", tim->CCR2);
  dbg_dec(  "  CCR3 = ", tim->CCR3);
  dbg_dec(  "  CCR4 = ", tim->CCR4);

  if (is_advanced) {
    dbg_hex32("  BDTR = ", tim->BDTR);
    dbg_print(  (tim->BDTR & TIM_BDTR_MOE) ? "    MOE: SET (master output enable)\r\n"
                                             : "    MOE: CLEAR *** NO OUTPUT ***\r\n");
    dbg_print(  (tim->BDTR & TIM_BDTR_AOE) ? "    AOE: SET (automatic output enable)\r\n"
                                             : "    AOE: CLEAR\r\n");
    dbg_print(  (tim->BDTR & TIM_BDTR_BKE) ? "    BKE: SET (break enabled)\r\n"
                                             : "    BKE: CLEAR (break disabled)\r\n");
    dbg_print(  (tim->BDTR & TIM_BDTR_OSSR) ? "    OSSR: SET\r\n"  : "    OSSR: CLEAR\r\n");
    dbg_print(  (tim->BDTR & TIM_BDTR_OSSI) ? "    OSSI: SET\r\n"  : "    OSSI: CLEAR\r\n");
  }
}

/* ------------------------------------------------------------------ */
/* 3. GPIO Register Dump for PWM pins                                 */
/* ------------------------------------------------------------------ */
static void dump_gpio_pin(const char *label, GPIO_TypeDef *port, uint32_t pin_num)
{
  dbg_print("  ");
  dbg_print(label);
  dbg_print(":\r\n");

  uint32_t moder = (port->MODER >> (pin_num * 2)) & 0x3;
  const char *mode_str;
  switch (moder) {
    case 0: mode_str = "INPUT";    break;
    case 1: mode_str = "OUTPUT";   break;
    case 2: mode_str = "AF";       break;
    case 3: mode_str = "ANALOG";   break;
    default: mode_str = "???";     break;
  }
  dbg_print("    MODER: ");
  dbg_print(mode_str);
  if (moder != 2) dbg_print(" *** SHOULD BE AF ***");
  dbg_print("\r\n");

  /* AF number */
  uint32_t af;
  if (pin_num < 8) {
    af = (port->AFR[0] >> (pin_num * 4)) & 0xF;
  } else {
    af = (port->AFR[1] >> ((pin_num - 8) * 4)) & 0xF;
  }
  dbg_dec("    AFR:   ", af);

  uint32_t otyper = (port->OTYPER >> pin_num) & 0x1;
  dbg_print(otyper ? "    OTYPER: OPEN-DRAIN *** SHOULD BE PP ***\r\n"
                    : "    OTYPER: PUSH-PULL (ok)\r\n");

  uint32_t ospeed = (port->OSPEEDR >> (pin_num * 2)) & 0x3;
  dbg_dec("    OSPEEDR: ", ospeed);

  uint32_t pupd = (port->PUPDR >> (pin_num * 2)) & 0x3;
  const char *pupd_str;
  switch (pupd) {
    case 0: pupd_str = "NONE";      break;
    case 1: pupd_str = "PULL-UP";   break;
    case 2: pupd_str = "PULL-DOWN"; break;
    default: pupd_str = "RESERVED";  break;
  }
  dbg_print("    PUPDR: ");
  dbg_print(pupd_str);
  dbg_print("\r\n");

  /* Read actual pin state (IDR) */
  uint32_t idr = (port->IDR >> pin_num) & 0x1;
  dbg_dec("    IDR (live): ", idr);
}

static void dump_gpio_all(void)
{
  dbg_print("\r\n=== GPIO PWM PIN CONFIG ===\r\n");

  /* TIM1: PA8=CH1(M1_RPWM), PA9=CH2(M1_LPWM), PA10=CH3(M2_RPWM), PA11=CH4(M2_LPWM) */
  dump_gpio_pin("PA8  TIM1_CH1 M1_RPWM", GPIOA, 8);
  dump_gpio_pin("PA9  TIM1_CH2 M1_LPWM", GPIOA, 9);
  dump_gpio_pin("PA10 TIM1_CH3 M2_RPWM", GPIOA, 10);
  dump_gpio_pin("PA11 TIM1_CH4 M2_LPWM", GPIOA, 11);

  /* TIM2: PA0=CH1(M3_RPWM), PA1=CH2(M3_LPWM) */
  dump_gpio_pin("PA0  TIM2_CH1 M3_RPWM", GPIOA, 0);
  dump_gpio_pin("PA1  TIM2_CH2 M3_LPWM", GPIOA, 1);

  /* EN_MOTOR: PB0 */
  dbg_print("\r\n=== EN_MOTOR PIN (PB0) ===\r\n");
  uint32_t pb0_moder = (GPIOB->MODER >> (0 * 2)) & 0x3;
  dbg_print(pb0_moder == 1 ? "  MODER: OUTPUT (ok)\r\n"
                            : "  MODER: *** NOT OUTPUT ***\r\n");
  uint32_t pb0_odr = (GPIOB->ODR >> 0) & 0x1;
  dbg_dec("  ODR (output): ", pb0_odr);
  uint32_t pb0_idr = (GPIOB->IDR >> 0) & 0x1;
  dbg_dec("  IDR (readback): ", pb0_idr);
}

/* ------------------------------------------------------------------ */
/* PUBLIC: Dump all static register state                             */
/* ------------------------------------------------------------------ */
void PWM_RegDebug_DumpAll(void)
{
  dbg_print("\r\n");
  dbg_print("########################################\r\n");
  dbg_print("# PWM REGISTER-LEVEL TRUTH VERIFICATION\r\n");
  dbg_print("########################################\r\n");

  dump_rcc();
  dump_tim_regs("TIM1 (M1+M2 PWM)", TIM1, 1);
  dump_tim_regs("TIM2 (M3 PWM)", TIM2, 0);
  dump_gpio_all();
}

/* ------------------------------------------------------------------ */
/* PUBLIC: Drive motor at 40% and dump live register changes          */
/* ------------------------------------------------------------------ */
void PWM_RegDebug_DriveAndDump(void)
{
  dbg_print("\r\n");
  dbg_print("########################################\r\n");
  dbg_print("# LIVE PWM DRIVE TEST (40% duty, 2s)   \r\n");
  dbg_print("########################################\r\n");

  /* === Phase 1: Start TIM1 PWM channels === */
  dbg_print("\r\n--- Starting TIM1 CH1+CH2 PWM ---\r\n");
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  dbg_print("After HAL_TIM_PWM_Start:\r\n");
  dbg_hex32("  TIM1->CR1  = ", TIM1->CR1);
  dbg_hex32("  TIM1->CCER = ", TIM1->CCER);
  dbg_hex32("  TIM1->BDTR = ", TIM1->BDTR);
  dbg_print(  (TIM1->BDTR & TIM_BDTR_MOE) ? "  MOE: SET\r\n" : "  MOE: CLEAR *** CRITICAL ***\r\n");
  dbg_print(  (TIM1->CR1  & TIM_CR1_CEN)  ? "  CEN: SET\r\n" : "  CEN: CLEAR *** CRITICAL ***\r\n");

  /* === Phase 2: Set CCR to 40% duty === */
  dbg_print("\r\n--- Setting M1: CCR1=4500, CCR2=0 ---\r\n");
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 4500);

  dbg_dec("  TIM1->CCR1 = ", TIM1->CCR1);
  dbg_dec("  TIM1->CCR2 = ", TIM1->CCR2);
  dbg_dec("  TIM1->ARR  = ", TIM1->ARR);

  /* Duty = CCR1/(ARR+1) = 4500/11250 = 40% */

  /* === Phase 3: Enable motor driver === */
  dbg_print("\r\n--- Asserting EN_MOTOR (PB0 = HIGH) ---\r\n");
  HAL_GPIO_WritePin(EN_MOTOR_GPIO_Port, EN_MOTOR_Pin, GPIO_PIN_SET);

  uint32_t en_odr = (GPIOB->ODR >> 0) & 0x1;
  uint32_t en_idr = (GPIOB->IDR >> 0) & 0x1;
  dbg_dec("  PB0 ODR = ", en_odr);
  dbg_dec("  PB0 IDR = ", en_idr);

  /* === Phase 4: Read live GPIO IDR on PWM pins (should be toggling) === */
  dbg_print("\r\n--- Reading PA8 IDR 10 times (100ms apart) ---\r\n");
  dbg_print("  (If PWM active, value should vary between reads)\r\n");
  for (int i = 0; i < 10; i++) {
    uint32_t pa8 = (GPIOA->IDR >> 8) & 0x1;
    uint32_t pa9 = (GPIOA->IDR >> 9) & 0x1;
    char line[48];
    int pos = 0;
    line[pos++] = ' '; line[pos++] = ' ';
    line[pos++] = 'P'; line[pos++] = 'A'; line[pos++] = '8'; line[pos++] = '=';
    line[pos++] = '0' + pa8;
    line[pos++] = ' '; line[pos++] = ' ';
    line[pos++] = 'P'; line[pos++] = 'A'; line[pos++] = '9'; line[pos++] = '=';
    line[pos++] = '0' + pa9;
    line[pos++] = '\r'; line[pos++] = '\n'; line[pos] = '\0';
    dbg_print(line);
    HAL_Delay(100);
  }

  /* === Phase 5: Read CCR after 1 second (verify still set) === */
  dbg_print("\r\n--- After 1 second of driving ---\r\n");
  dbg_dec("  TIM1->CCR1 = ", TIM1->CCR1);
  dbg_dec("  TIM1->CCR2 = ", TIM1->CCR2);
  dbg_hex32("  TIM1->CR1  = ", TIM1->CR1);
  dbg_hex32("  TIM1->BDTR = ", TIM1->BDTR);
  dbg_hex32("  TIM1->CCER = ", TIM1->CCER);

  /* === Phase 6: Read GPIOA->MODER one more time (detect late reconfiguration) === */
  dbg_print("\r\n--- GPIOA->MODER check (late reconfiguration?) ---\r\n");
  dbg_hex32("  GPIOA->MODER = ", GPIOA->MODER);
  /* PA8 should be bits [17:16] = 0b10 (AF) = 0x2 */
  uint32_t pa8_mode = (GPIOA->MODER >> (8*2)) & 0x3;
  uint32_t pa9_mode = (GPIOA->MODER >> (9*2)) & 0x3;
  dbg_dec("  PA8 MODER = ", pa8_mode);
  dbg_dec("  PA9 MODER = ", pa9_mode);
  if (pa8_mode != 2 || pa9_mode != 2) {
    dbg_print("  *** CRITICAL: PWM PINS NOT IN AF MODE ***\r\n");
  }

  /* === Cleanup === */
  dbg_print("\r\n--- Stopping ---\r\n");
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  HAL_GPIO_WritePin(EN_MOTOR_GPIO_Port, EN_MOTOR_Pin, GPIO_PIN_RESET);

  dbg_print("\r\n=== DRIVE TEST COMPLETE ===\r\n");
  dbg_print("If motor did NOT twitch during this 2-second window:\r\n");
  dbg_print("  Check: BTS7960 VM/B+ power supply\r\n");
  dbg_print("  Check: PB0 wire to BTS7960 R_EN + L_EN\r\n");
  dbg_print("  Check: PA8 wire to BTS7960 R_PWM\r\n");
  dbg_print("  Check: PA9 wire to BTS7960 L_PWM\r\n");
  dbg_print("  Check: BTS7960 motor output to motor terminals\r\n");
}
