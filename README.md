# Parking-Sensor-STM32

## Overview
This project is a parking sensor system built using an **STM32F446RE** microcontroller, an **HC-SR04** ultrasonic distance sensor, an **LCD1602A** display with its **I2C converter module**, a buzzer, and three LEDs (red, yellow, green) to indicate distance levels. The system measures the distance to an obstacle and displays the value on the LCD screen in centimeters. Additionally, it provides visual and auditory feedback based on the proximity of the obstacle.

## Components
- **STM32F446RE**
- **HC-SR04 Ultrasonic Sensor**
- **LCD1602A with I2C Module**: Displays the distance measured by the HC-SR04.
- **Buzzer**: Provides auditory feedback when an obstacle is detected within a critical distance.
- **LEDs**: Three different colored LEDs (red, yellow, green) indicate the distance level:

- **Breadboard**: Used to connect and organize the components.

## Functionality
1. **Distance Measurement**: 
   - The HC-SR04 sensor measures the distance to an obstacle by sending an ultrasonic pulse and timing how long it takes to receive the echo. 
   - The distance is calculated by using the speed of sound and the time difference between the transmitted and received pulses.
   
2. **Timer Configuration**:
   - **TMR3** on the STM32F446RE is configured in **input capture mode** to accurately measure the time intervals for distance calculation.
   - On the rising edge of the pulse, the timestamp is recorded, and the timer is reconfigured to capture the falling edge. The difference between the two timestamps gives the time interval, which is then used to calculate the distance.

3. **Distance Display**:
   - The calculated distance is displayed in centimeters on the LCD1602A display. In order to interface LCD1602A, its **I2C serial communication module** is used along with its driver.

4. **LED and Buzzer Indication**:
   - The LEDs and the buzzer are used to indicate the distance level.

## How It Works
- The system continuously triggers the HC-SR04 with 250ms time intervals to measure the distance.
- The STM32F446RE uses **interrupts** to handle the input capture from the HC-SR04 sensor, ensuring that the main program can run **smoothly** without being blocked by sensor reading operations.
- The measured distance is processed and displayed on the LCD, and the LEDs and buzzer provide immediate feedback based on the proximity to an obstacle.

## Key Features
- **Accurate Distance Measurement**: Using timer-based input capture, the system measures distances with high precision.
- **Non-blocking Operation**: The use of interrupts ensures that the main program can continue executing other tasks while distance measurements are being processed.
- **Real-time Feedback**: The system provides real-time visual and auditory feedback based on the measured distance.

## Code Snippet
```c
void handleTMR4Interrupt(TIM_HandleTypeDef *htim) {

	if (inputCaptureState == ON_RISING_EDGE)
	{
		captureTimeOnRiseEdge(htim);
	}

	else if (inputCaptureState == ON_FALLING_EDGE)
	{
		captureTimeOnFallEdge(htim);
	}
}

void handle_HCSR04Interrupt() {
    HCSR04_ReadInput();
    enum DistanceLevel level = getDistanceLevel(measuredDistance);

    char buffer[16];
    sprintf(buffer, "Dist: %dcm", measuredDistance);
    HD44780_Clear();
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr(buffer);

    toggleLEDs(level);
    toggleBuzzer(level);
}
```

## Conclusion
This parking sensor project is a practical application of the STM32 microcontroller, demonstrating real-time distance measurement and feedback using an ultrasonic sensor, LEDs, and a buzzer. The use of input capture interrupts ensures efficient processing, allowing for smooth operation and accurate distance detection.