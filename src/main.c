/**
 * @file   main.c
 * @brief  Robotic arm SDK peripheral.
 *         Nucleo-F401RE STM32 Board.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>

#define USART_BAUDRATE (9600)
#define BUTTON_SEND_DATA ((uint8_t)0xF0)
#define DELAY_VALUE ((uint32_t)250000)
#define DEBOUNCE_DELAY_VALUE ((uint32_t)20000)

#define USB_USART (USART2)
#define RCC_USB_USART (RCC_USART2)

/* USART-Tx. */
#define USART_TX_PORT (GPIOA)
#define USART_TX_PIN (GPIO2)

/* USART-Rx. */
#define USART_RX_PORT (GPIOA)
#define USART_RX_PIN (GPIO3)

/* User LED. */
#define LED_PORT (GPIOA)
#define LED_PIN (GPIO5)

/* User Button. */
#define BUTTON_PORT (GPIOC)
#define BUTTON_PIN (GPIO13)
#define BUTTON_EXTI (EXTI13)

/* D8. */
#define D8_PORT (GPIOA)
#define D8_PIN (GPIO9)

/* D9. */
#define D9_PORT (GPIOC)
#define D9_PIN (GPIO7)

/* D10. */
#define D10_PORT (GPIOB)
#define D10_PIN (GPIO6)
  
/* ASCII Table. */
#define ASCII_CR ((uint8_t)0x0D) // CR: \r.
#define ASCII_LF ((uint8_t)0x0A) // LF: \n.
#define ASCII_DEL ((uint8_t)0x7F)

void rcc_setup(void);
void usart_setup(void);
void led_setup(void);
void others_gpio_setup(void);
void button_setup(void);
void delay(volatile unsigned int value);

int main(void)
{
  /* Init. */
  rcc_setup();
  led_setup();
  others_gpio_setup();
  button_setup();
  usart_setup();

  /* Sned 'Ready'. */
  usart_send_blocking(USB_USART, 'R');
  usart_send_blocking(USB_USART, 'e');
  usart_send_blocking(USB_USART, 'a');
  usart_send_blocking(USB_USART, 'd');
  usart_send_blocking(USB_USART, 'y');
  usart_send_blocking(USB_USART, '\r');
  usart_send_blocking(USB_USART, '\n');

  /* Into main loop and wait for ISRs. */
  while (1)
  {
    /* LED on/off. */
    gpio_toggle(LED_PORT, LED_PIN);
    delay(DELAY_VALUE);
  }

  return 0;
}

void rcc_setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);

  rcc_periph_clock_enable(RCC_USB_USART);
  rcc_periph_clock_enable(RCC_SYSCFG); // For EXTI.
}

void usart_setup(void)
{
  /* Enable USART IRQ. */
  nvic_enable_irq(NVIC_USART2_IRQ);

  /* Setup Tx pin. */
  gpio_mode_setup(USART_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART_TX_PIN);
  gpio_set_output_options(USART_TX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, USART_TX_PIN);
  gpio_set_af(USART_TX_PORT, GPIO_AF7, USART_TX_PIN);

  /* Setup Rx pin. */
  gpio_mode_setup(USART_RX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART_RX_PIN);
  gpio_set_af(USART_RX_PORT, GPIO_AF7, USART_RX_PIN);

  /* Setup USART config. */
  usart_set_baudrate(USB_USART, USART_BAUDRATE);
  usart_set_databits(USB_USART, 8);
  usart_set_stopbits(USB_USART, USART_STOPBITS_1);
  usart_set_parity(USB_USART, USART_PARITY_NONE);
  usart_set_flow_control(USB_USART, USART_FLOWCONTROL_NONE);
  usart_set_mode(USB_USART, USART_MODE_TX_RX);

  /* Enable Rx interrupt. */
  usart_enable_rx_interrupt(USB_USART);

  /* Enable. */
  usart_enable(USB_USART);
}

void led_setup(void)
{
  gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
  gpio_set_output_options(LED_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, LED_PIN);
}

void others_gpio_setup()
{
  gpio_mode_setup(D8_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, D8_PIN);
  gpio_set_output_options(D8_PIN, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, D8_PORT);
  gpio_set(D8_PORT, D8_PIN);

  gpio_mode_setup(D9_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, D9_PIN);
  gpio_set_output_options(D9_PIN, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, D9_PORT);
  gpio_set(D9_PORT, D9_PIN);

  gpio_mode_setup(D10_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, D10_PIN);
  gpio_set_output_options(D10_PIN, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, D10_PORT);
  gpio_set(D10_PORT, D10_PIN);
}

void button_setup(void)
{
  nvic_enable_irq(NVIC_EXTI15_10_IRQ);

  gpio_mode_setup(BUTTON_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BUTTON_PIN);

  exti_select_source(BUTTON_EXTI, BUTTON_PORT);
  exti_set_trigger(BUTTON_EXTI, EXTI_TRIGGER_FALLING);
  exti_enable_request(BUTTON_EXTI);
}

void delay(volatile unsigned int value)
{
  while (value--)
  {
    __asm__("nop"); // Do nothing.
  }
}

/**
 * @brief EXTI15~10 Interrupt service routine.
 * @remark User button.
 */
void exti15_10_isr(void)
{
  // Clear flag first.
  exti_reset_request(BUTTON_EXTI);

  // Simple button debounce.
  delay(DEBOUNCE_DELAY_VALUE);
  if(gpio_get(BUTTON_PORT, BUTTON_PIN) == 0)
  {
    usart_send_blocking(USB_USART, BUTTON_SEND_DATA);
    usart_send_blocking(USB_USART, ASCII_CR);
    usart_send_blocking(USB_USART, ASCII_LF);
  }
}

/**
 * @brief USART2 Interrupt service routine.
 * @remark USB serial port.
 */
void usart2_isr(void)
{
  uint8_t indata = usart_recv(USB_USART);
  switch (indata)
  {
  case 0x00:
    gpio_set(D8_PORT, D8_PIN);
    break;

  case 0x01:
    gpio_clear(D8_PORT, D8_PIN);
    break;

  case 0x10:
    gpio_set(D9_PORT, D9_PIN);
    break;

  case 0x11:
    gpio_clear(D9_PORT, D9_PIN);
    break;

  case 0x20:
    gpio_set(D10_PORT, D10_PIN);
    break;

  case 0x21:
    gpio_clear(D10_PORT, D10_PIN);
    break;

  case ASCII_DEL:
    gpio_set(D8_PORT, D8_PIN);
    gpio_set(D9_PORT, D9_PIN);
    gpio_set(D10_PORT, D10_PIN);
    break;

  case ASCII_CR:
  case ASCII_LF:
  default:
    /* Do nothing. */
    break;

  }

  /*
   * Clear RXNE(Read data register not empty) flag of
   * USART SR(Status register).
   */
  USART_SR(USB_USART) &= ~USART_SR_RXNE;
}
