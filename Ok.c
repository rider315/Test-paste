#include <stdlib.h>
#include <string.h>

GPIO->MODER |= (1<<6);
GPIO->MODER |= (1<<7);
GPIO->MODER |= (1<<8);  // Configure LD1, LD2, LD3 as output

int fuelstore[3] = {30, 15, 2};
char buffer[10];  // Buffer for up to 9 digits + null terminator
uint8_t fuel_level = 0;

void send_uart_message(const char* message) {
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
}

void receive_uart_input(char* buffer, uint8_t max_len) {
    uint8_t index = 0;
    uint8_t byte;

    while (index < max_len - 1) {  // Leave space for null terminator
        if (HAL_UART_Receive(&hlpuart1, &byte, 1, 1000) == HAL_OK) {  // Receive 1 byte at a time, 1s timeout
            if (byte == '\n' || byte == '\r') {  // Stop at newline or carriage return
                buffer[index] = '\0';  // Null terminate the string
                break;
            }
            buffer[index++] = byte;  // Add byte to buffer
        }
    }
    buffer[index] = '\0';  // Ensure null termination
}

while (1) {
    // Clear buffer
    memset(buffer, 0, sizeof(buffer));

    // Receive input until newline
    receive_uart_input(buffer, sizeof(buffer));

    // Process the input if we received something
    if (strlen(buffer) > 0) {
        char *endptr;
        long temp = strtol(buffer, &endptr, 10);  // Convert string to long

        // Check if conversion was successful and value is within uint8_t range
        if (endptr != buffer && temp >= 0 && temp <= 255) {
            fuel_level = (uint8_t)temp;

            // Reset all LEDs
            GPIO->ODR &= ~(1<<3);  // LD1 off
            GPIO->ODR &= ~(1<<4);  // LD2 off
            GPIO->ODR &= ~(1<<5);  // LD3 off

            if (fuel_level >= fuelstore[0]) {  // >= 30L
                send_uart_message("Fuel level: Full\r\n");
                GPIO->ODR |= (1<<3);  // Turn on LD1
                send_uart_message("Full tank\r\n");
            }
            else if (fuel_level >= fuelstore[1] && fuel_level < fuelstore[0]) {  // >= 15L and < 30L
                send_uart_message("Fuel level: Half\r\n");
                GPIO->ODR |= (1<<4);  // Turn on LD2
                send_uart_message("Half tank\r\n");
            }
            else if (fuel_level < fuelstore[2]) {  // < 2L
                send_uart_message("Fuel level: Empty\r\n");
                GPIO->ODR |= (1<<5);  // Turn on LD3
                send_uart_message("Empty tank\r\n");
            }
        }
        else {
            send_uart_message("Invalid input\r\n");
        }
    }
}
