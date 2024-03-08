#ifndef COMMON_CLOCK_CONTROL_H
#define COMMON_CLOCK_CONTROL_H

#if VERSION_MAJOR >= 4

#include "looper.h"

class ClockControl : public Looper {
public:
  const char* name() override { return "ClockControl"; }
  void Loop() override {
    bool on = false;
    SaberBase::DoIsOn(&on);
	
    if (SaberBase::IsOn()
	|| Serial
	|| prop.NeedsPower()
	|| USBD_Connected()
	|| (stm32l4_gpio_pin_read(GPIO_PIN_PB2) && on == false)
#ifdef ENABLE_AUDIO
	|| amplifier.Active()
#endif
      ) {
      last_activity_ = millis();
    }
    // These two variables must be read in order.
    uint32_t last_activity = last_activity_;
    uint32_t now = millis();

	// Allow motion override to take place prior to sleep activation
	if(now - last_activity > 28000) {
		SaberBase::OverrideMotionSleepRequested(true);	
	} else {
		SaberBase::OverrideMotionSleepRequested(false);
	}

    if (now - last_activity > 30000) {
      uint32_t pclk1 = stm32l4_system_pclk1();
      uint32_t pclk2 = stm32l4_system_pclk2();
#if 0 // #ifdef PROFFIEBOARD_VERSION
      // This saves power, but also casuses freezing.
      // TODO: FIgure out why and re-enable.
      stm32l4_system_sysclk_configure(1000000, 500000, 500000);
#else

// Clock transition no longer necessary
//      stm32l4_system_sysclk_configure(16000000, 8000000, 8000000);

	  // Briefly enable booster to allow it to properly enter shutdown mode
	  EnableBooster();
	  delay(500);

	  // TODO: Add Proffie V1 pins (if different than V2)
#if PROFFIEBOARD_VERSION==2
	  // Update GPIO pin states (needs to be streamlined at some point, not all necessary)
	  stm32l4_gpio_pin_configure(GPIO_PIN_PA0, GPIO_MODE_ANALOG);
	  stm32l4_gpio_pin_configure(GPIO_PIN_PA1, GPIO_MODE_ANALOG);
	  stm32l4_gpio_pin_configure(GPIO_PIN_PA2, GPIO_MODE_ANALOG);
	  stm32l4_gpio_pin_configure(GPIO_PIN_PA3, GPIO_MODE_ANALOG); // vtest
	  stm32l4_gpio_pin_configure(GPIO_PIN_PA4, GPIO_MODE_ANALOG);
	  stm32l4_gpio_pin_configure(GPIO_PIN_PA5, GPIO_MODE_ANALOG);
	  stm32l4_gpio_pin_configure(GPIO_PIN_PA6, GPIO_MODE_ANALOG);
	  stm32l4_gpio_pin_configure(GPIO_PIN_PA7, GPIO_MODE_ANALOG);
	  stm32l4_gpio_pin_configure(GPIO_PIN_PA8, GPIO_MODE_ANALOG);
	  stm32l4_gpio_pin_configure(GPIO_PIN_PA9, GPIO_MODE_ANALOG); // SCL
	  stm32l4_gpio_pin_configure(GPIO_PIN_PA10, GPIO_MODE_ANALOG);
	  stm32l4_gpio_pin_configure(GPIO_PIN_PA11, GPIO_MODE_ANALOG);
	  stm32l4_gpio_pin_configure(GPIO_PIN_PA12, GPIO_MODE_ANALOG);
	  stm32l4_gpio_pin_configure(GPIO_PIN_PA13, GPIO_MODE_ANALOG);
	  stm32l4_gpio_pin_configure(GPIO_PIN_PB0, GPIO_MODE_ANALOG);
	  stm32l4_gpio_pin_configure(GPIO_PIN_PB1, GPIO_MODE_ANALOG);
//	  stm32l4_gpio_pin_configure(GPIO_PIN_PB2, GPIO_MODE_ANALOG); // VBUS
	  stm32l4_gpio_pin_configure(GPIO_PIN_PB3, GPIO_MODE_ANALOG); // Free2
//	  stm32l4_gpio_pin_configure(GPIO_PIN_PB4, GPIO_MODE_ANALOG); // Button 3
//	  stm32l4_gpio_pin_configure(GPIO_PIN_PB5, GPIO_MODE_ANALOG); // Button 2
//	  stm32l4_gpio_pin_configure(GPIO_PIN_PB6, GPIO_MODE_ANALOG); // Button 1
	  stm32l4_gpio_pin_configure(GPIO_PIN_PB7, GPIO_MODE_ANALOG);
	  stm32l4_gpio_pin_configure(GPIO_PIN_PB8, GPIO_MODE_ANALOG);
	  stm32l4_gpio_pin_configure(GPIO_PIN_PB9, GPIO_MODE_ANALOG); // SDA
	  stm32l4_gpio_pin_configure(GPIO_PIN_PB10, GPIO_MODE_ANALOG);
	  stm32l4_gpio_pin_configure(GPIO_PIN_PB11, GPIO_MODE_ANALOG);
	  stm32l4_gpio_pin_configure(GPIO_PIN_PB12, GPIO_MODE_ANALOG);
	  stm32l4_gpio_pin_configure(GPIO_PIN_PB13, GPIO_MODE_ANALOG);
	  stm32l4_gpio_pin_configure(GPIO_PIN_PB14, GPIO_MODE_ANALOG);
	  stm32l4_gpio_pin_configure(GPIO_PIN_PB15, GPIO_MODE_ANALOG);
	  stm32l4_gpio_pin_configure(GPIO_PIN_PC13, GPIO_MODE_ANALOG); // Int
//	  stm32l4_gpio_pin_configure(GPIO_PIN_PH0, GPIO_MODE_ANALOG); // Boost
//	  stm32l4_gpio_pin_configure(GPIO_PIN_PH1, GPIO_MODE_ANALOG); // Amp
#endif
#if PROFFIEBOARD_VERSION==3
	  // TODO: Add Proffie 3 pins
#endif

	  // Play beep indicating that low power sleep was entered
	  beeper.Beep(0.7, 500);
	  delay(1000);

	  // Make sure booster and amplifier pins are LOW before sleep to activate shutdown modes
	  digitalWrite(boosterPin, LOW);
	  digitalWrite(amplifierPin, LOW);

	  // Force peripherals into standby
	  stm32l4_system_periph_disable(SYSTEM_PERIPH_I2C1);
	  stm32l4_system_periph_disable(SYSTEM_PERIPH_I2C2);
	  stm32l4_system_periph_disable(SYSTEM_PERIPH_I2C3);
	  stm32l4_system_periph_disable(SYSTEM_PERIPH_SPI1);
	  stm32l4_system_periph_disable(SYSTEM_PERIPH_SPI2);
	  stm32l4_system_periph_disable(SYSTEM_PERIPH_SPI3);	  
	  stm32l4_system_periph_disable(SYSTEM_PERIPH_USART1);
	  stm32l4_system_periph_disable(SYSTEM_PERIPH_USART2);
	  stm32l4_system_periph_disable(SYSTEM_PERIPH_USART3);
	  stm32l4_system_periph_disable(SYSTEM_PERIPH_LPUART1);

	  // Loop indefinitely, only exitting once wake conditions present
	  while(1) {

		  // Activate STOP2 sleep mode
		  MODIFY_REG(PWR->CR1, PWR_CR1_LPMS, PWR_CR1_LPMS_STOP2);
		  STM32.stop(1000 * 4); // Sleep for X seconds

		  // Todo: Add in dynamic voltage check to determine if booster should be enabled

		  // Check for wake-up conditions each time through loop
		  if(USBD_Connected()
		  || stm32l4_gpio_pin_read(GPIO_PIN_PB5) == 0 // Button 1
		  || stm32l4_gpio_pin_read(GPIO_PIN_PB6) == 0 // Button 2
		  || stm32l4_gpio_pin_read(GPIO_PIN_PB2) == 1 // VBUS
		  ) {
			
			  // If USB is NOT present, play beep indicating sleep mode exitted
			  if(USBD_Connected() == FALSE
			  && !stm32l4_gpio_pin_read(GPIO_PIN_PB2)) {

				  // Play a short beep indicating that the board is waking up
				  EnableBooster();
				  delay(250);
				  digitalWrite(amplifierPin, HIGH);
				  beeper.Beep(0.7, 1000);
				  delay(2000);
			  }
			
			  // Reset processor instead of attempting to resume where left off
			  // TODO: Could be augmented to allow seamless wake-up someday
			  STM32.reset();
			  break;
		  }
	  }

#endif
#ifdef COMMON_I2CBUS_H
      // Motion and other things might still be going on.
      if (i2cbus.used())
        delay(5);
      else
#endif
        delay(50);
      stm32l4_system_sysclk_configure(_SYSTEM_CORE_CLOCK_, pclk1, pclk2);
    }
  }

  // Do nothing now, override font request
  void AvoidSleep() { }
    
  private:
    volatile uint32_t last_activity_;
};

ClockControl clock_control;

void ClockControl_AvoidSleep() { clock_control.AvoidSleep(); }

#else

void ClockControl_AvoidSleep() { }

#endif

#endif
