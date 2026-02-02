#include "uart_handler.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// Include main.h to get macro definitions
#include "main.h"



// Khoi tao UART handler
void UartInit(UartHandler *handler, UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_rx) {
    handler->huart = huart;
    handler->hdma_rx = hdma_rx;

    memset(handler->rxBuffer, 0, UART_RX_BUFFER_SIZE);
    memset(handler->rxProcessBuffer, 0, UART_RX_BUFFER_SIZE);
    memset(handler->rxLineBuffer, 0, UART_RX_BUFFER_SIZE);
    handler->rxLineSize = 0;
    handler->oldPos = 0;
    handler->newPos = 0;
    handler->lineReady = false;
    handler->txBusy = false;

    // Initialize UART type flags
    handler->is_camera_uart = (huart->Instance == USART1); // UART1 for camera
    handler->is_debug_uart = (huart->Instance == USART2);  // UART2 for debug

    
    // Initialize camera specific data
    if (handler->is_camera_uart) {
        memset(&handler->camera_buffer, 0, sizeof(CameraDataBuffer));
        memset(handler->camera_frame_buffer, 0, CAMERA_MAX_FRAME_SIZE);  // Clear full buffer
        handler->camera_frame_pos = 0;
        handler->target_frame_ready = false;  // Initialize target frame flag
        
        // Trajectory defaults
        handler->trajectory.mode = TRAJECTORY_MODE_NONE;
        handler->trajectory.active = false;
        handler->trajectory.start_time = 0;
        handler->trajectory.radius_cm = 0.0f;
        handler->trajectory.speed_rad_s = 0.0f;
        handler->trajectory.target_x = 0.0f;
        handler->trajectory.target_y = 0.0f;
        handler->trajectory_frame_ready = false;
    }
    

    


    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);  // Ngat khi IDLE
}



// Bat dau nhan DMA vong
void UartStartReceive(UartHandler *handler) {
    HAL_UART_Receive_DMA(handler->huart, handler->rxBuffer, UART_RX_BUFFER_SIZE);
}

// Xu ly chuoi du lieu nhan duoc -> chi cap nhat vi tri bong (manual input only)
bool UartProcessData(UartHandler *handler, BallPosition *ball) {
    if (!handler->lineReady) return false;
    handler->rxLineBuffer[handler->rxLineSize] = '\0';
    
    char *line = (char *)handler->rxLineBuffer;
    
    // Only process coordinate data: "x,y" format
    char *comma = strchr(line, ',');
    if (comma == NULL) {
        handler->lineReady = false;
        handler->rxLineSize = 0;
        return false;
    }
    *comma = '\0';
    float x = atof(line);
    float y = atof(comma + 1);
    BallPositionUpdate(ball, x, y);
    handler->lineReady = false;
    handler->rxLineSize = 0;
    
    return true;
}

// Process target coordinate frame (for setting PID target)  
bool UartProcessTargetFrame(UartHandler *handler, float *target_x, float *target_y) {
    if (!handler->is_camera_uart) return false;

    // PRIORITY PROCESSING: Check if target frame is ready for immediate processing
    if (!handler->target_frame_ready) {
        return false;  // No target frame ready, skip processing
    }

    uint8_t *frame = handler->camera_frame_buffer;

    // Process the ready target frame
    if (frame[0] == CAMERA_FRAME_START && frame[1] == 'T' && frame[4] == CAMERA_FRAME_STOP) {
        // Calculate checksum from START to STOP (5 bytes) 
        uint16_t calculated_checksum = CalculateChecksum(frame, 5);
        uint16_t received_checksum = frame[5] | ((uint16_t)frame[6] << 8); // Little endian
        
        if (calculated_checksum == received_checksum) {
            // Extract target coordinates (int8 format)
            int8_t x_int = (int8_t)frame[2];
            int8_t y_int = (int8_t)frame[3];
            
            // Convert to float (divide by scale factor)
            *target_x = (float)x_int / COORDINATE_SCALE;
            *target_y = (float)y_int / COORDINATE_SCALE;

            // Validate range
            if (ValidateCameraData(*target_x, *target_y)) {
                // Clear the frame buffer and reset flag to prevent re-processing
                memset(frame, 0, TARGET_FRAME_SIZE);
                handler->target_frame_ready = false;  // Reset priority flag
                return true;
            }
        }
    }

    // If processing failed, reset flag to prevent repeated attempts
    handler->target_frame_ready = false;
    return false;
}

// Process trajectory control frames: circle command
bool UartProcessTrajectoryFrame(UartHandler *handler) {
    if (!handler->is_camera_uart) return false;

    if (!handler->trajectory_frame_ready) return false;

    uint8_t *frame = handler->camera_frame_buffer;

    // Circle: @ C <radius_int16> <speed_int16> X <chksum>
    if (frame[0] == CAMERA_FRAME_START && frame[1] == 'C' && frame[6] == CAMERA_FRAME_STOP) {
        // checksum over bytes 0..6 (start to stop)
        uint16_t calc = CalculateChecksum(frame, 7);
        uint16_t recv = frame[7] | ((uint16_t)frame[8] << 8);
        if (calc == recv) {
            int16_t r_i = (int16_t)(frame[2] | ((uint16_t)frame[3] << 8));
            int16_t w_i = (int16_t)(frame[4] | ((uint16_t)frame[5] << 8));
            // Protocol units: radius in mm (unit=1 mm) => 0.1 cm per unit if desired
            float radius_cm = ((float)r_i) / 10.0f;
            float speed_rad_s = ((float)w_i) / 1000.0f;  // mrad/s -> rad/s

            if (radius_cm < 0) radius_cm = -radius_cm;
            if (radius_cm > 9.5f) radius_cm = 9.5f;

            handler->trajectory.mode = TRAJECTORY_MODE_CIRCLE;
            handler->trajectory.radius_cm = radius_cm;
            handler->trajectory.speed_rad_s = speed_rad_s;
            handler->trajectory.active = true;
            handler->trajectory.start_time = HAL_GetTick();

            handler->trajectory_frame_ready = false;
            memset(frame, 0, TRAJECTORY_CIRCLE_SIZE);
            return true;
        }
    }
    // Stop: @ S X <chksum>
    else if (frame[0] == CAMERA_FRAME_START && frame[1] == 'S' && frame[2] == CAMERA_FRAME_STOP) {
        uint16_t calc = CalculateChecksum(frame, 3);
        uint16_t recv = frame[3] | ((uint16_t)frame[4] << 8);
        if (calc == recv) {
            UartStopTrajectory(handler);
            handler->trajectory_frame_ready = false;
            memset(frame, 0, TRAJECTORY_STOP_SIZE);
            return true;
        }
    }
    // Square: @ Q <side_int16> X <chksum>
    else if (frame[0] == CAMERA_FRAME_START && frame[1] == 'Q' && frame[4] == CAMERA_FRAME_STOP) {
        // checksum over bytes 0..4 (start to stop)
        uint16_t calc = CalculateChecksum(frame, 5);
        uint16_t recv = frame[5] | ((uint16_t)frame[6] << 8);
        if (calc == recv) {
            int16_t side_mm = (int16_t)(frame[2] | ((uint16_t)frame[3] << 8));
            float side_cm = ((float)side_mm) / 10.0f;  // mm -> cm
            
            if (side_cm < 0) side_cm = -side_cm;
            if (side_cm > 9.5f) side_cm = 9.5f;
            
            // Note: Square trajectory mode not implemented in main loop yet
            // For now, just stop trajectory to prevent errors
            UartStopTrajectory(handler);
            
            handler->trajectory_frame_ready = false;
            memset(frame, 0, TRAJECTORY_SQUARE_SIZE);
            return true;
        }
    }

    // Unknown or failed processing
    handler->trajectory_frame_ready = false;
    return false;
}

// Update trajectory position based on current time
void UartUpdateTrajectory(UartHandler *handler) {
    if (!handler->is_camera_uart) return;
    if (!handler->trajectory.active) return;

    if (handler->trajectory.mode == TRAJECTORY_MODE_CIRCLE) {
        uint32_t now = HAL_GetTick();
        float t = (now - handler->trajectory.start_time) / 1000.0f;
        float angle = handler->trajectory.speed_rad_s * t;
        handler->trajectory.target_x = handler->trajectory.radius_cm * cosf(angle);
        handler->trajectory.target_y = handler->trajectory.radius_cm * sinf(angle);
    }
}

// Stop trajectory
void UartStopTrajectory(UartHandler *handler) {
    handler->trajectory.active = false;
    handler->trajectory.mode = TRAJECTORY_MODE_NONE;
}



// Truyen du lieu bang DMA
void UartTransmit(UartHandler *handler, const char *data, uint16_t size) {
    uint32_t timeout = HAL_GetTick() + 100;

    while (handler->txBusy) {
        if (HAL_GetTick() > timeout) {
            HAL_UART_AbortTransmit(handler->huart);
            handler->txBusy = false;
            break;
        }
        HAL_Delay(1);
    }

    uint16_t copySize = (size < UART_TX_BUFFER_SIZE) ? size : UART_TX_BUFFER_SIZE - 1;
    memcpy(handler->txBuffer, data, copySize);
    handler->txBusy = true;

    // Try DMA transmission first
    HAL_StatusTypeDef dma_status = HAL_UART_Transmit_DMA(handler->huart, handler->txBuffer, copySize);

    // If DMA fails, use blocking transmission as fallback
    if (dma_status != HAL_OK) {
        handler->txBusy = false;  // Reset busy flag
        HAL_UART_Transmit(handler->huart, handler->txBuffer, copySize, 1000);
    }
}

// Truyen chuoi null-terminated
void UartTransmitString(UartHandler *handler, const char *str) {
    UartTransmit(handler, str, strlen(str));
}

// Callback khi DMA RX hoan tat (goi tu HAL_UART_RxCpltCallback)
void UartDmaRxCpltCallback(UartHandler *handler) {
    UartCheckDMA(handler);
}

// Callback khi DMA TX hoan tat (goi tu HAL_UART_TxCpltCallback)
void UartDmaTxCpltCallback(UartHandler *handler) {
    handler->txBusy = false;
}

// Kiem tra DMA buffer xem co du lieu moi khong
bool UartCheckDMA(UartHandler *handler) {  //Lấy feedback từ camera qua UART1
    handler->newPos = UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(handler->hdma_rx);

    if (handler->newPos != handler->oldPos) {
        // For camera UART (UART1), keep existing copy-then-process behavior
        if (handler->is_camera_uart) {
            if (handler->newPos > handler->oldPos) {
                memcpy(handler->rxProcessBuffer, &handler->rxBuffer[handler->oldPos], handler->newPos - handler->oldPos);
            } else {
                memcpy(handler->rxProcessBuffer, &handler->rxBuffer[handler->oldPos], UART_RX_BUFFER_SIZE - handler->oldPos);
                memcpy(&handler->rxProcessBuffer[UART_RX_BUFFER_SIZE - handler->oldPos], handler->rxBuffer, handler->newPos);
            }

            // Process the newly received chunk of data
            UartProcessBuffer(handler, handler->newPos > handler->oldPos ? (handler->newPos - handler->oldPos) : (UART_RX_BUFFER_SIZE - handler->oldPos + handler->newPos));

            // Advance oldPos only for camera UART
            handler->oldPos = handler->newPos;
            return true; // Indicate that new data was found
        }

        // For debug UART (UART2), do not consume bytes here.
        // Leave rxBuffer indices intact so UartProcessPsoFrame() can parse
        // frames directly from the ring buffer without losing data.
        return true;
    }
    return false; // No new data
}

// Xu ly du lieu nhan de lay chuoi ket thuc bang CR hoac LF
void UartProcessBuffer(UartHandler *handler, uint16_t len) {
    if (handler->is_camera_uart) {
        // Process binary camera frames
        for (uint16_t i = 0; i < len; i++) {
            uint8_t byte = handler->rxProcessBuffer[i];
            
            if (byte == CAMERA_FRAME_START) {
                // Start new frame
                handler->camera_frame_pos = 0;
                handler->camera_frame_buffer[handler->camera_frame_pos++] = byte;
            } else if (handler->camera_frame_pos > 0) {
                // Continue building frame
                if (handler->camera_frame_pos < CAMERA_MAX_FRAME_SIZE) {  // Max buffer size
                    handler->camera_frame_buffer[handler->camera_frame_pos++] = byte;
                }

                // Check if frame is complete (prioritize control frames first)
                if (handler->camera_frame_pos == TARGET_FRAME_SIZE && handler->camera_frame_buffer[1] == 'T') {
                    // Target frame (7 bytes) - priority
                    handler->target_frame_ready = true;
                    handler->camera_frame_pos = 0;
                } else if (handler->camera_frame_pos == TRAJECTORY_CIRCLE_SIZE && handler->camera_frame_buffer[1] == 'C') {
                    // Circle trajectory frame (9 bytes) - priority
                    handler->trajectory_frame_ready = true;
                    handler->camera_frame_pos = 0;
                } else if (handler->camera_frame_pos == TRAJECTORY_STOP_SIZE && handler->camera_frame_buffer[1] == 'S') {
                    // Stop trajectory frame (5 bytes) - priority
                    handler->trajectory_frame_ready = true;
                    handler->camera_frame_pos = 0;
                } else if (handler->camera_frame_pos == TRAJECTORY_SQUARE_SIZE && handler->camera_frame_buffer[1] == 'Q') {
                    // Square trajectory frame (7 bytes)
                    handler->trajectory_frame_ready = true;
                    handler->camera_frame_pos = 0;
                } else if (handler->camera_frame_pos == CAMERA_FRAME_SIZE &&
                           handler->camera_frame_buffer[1] != 'T' &&
                           handler->camera_frame_buffer[1] != 'C' &&
                           handler->camera_frame_buffer[1] != 'S' &&
                           handler->camera_frame_buffer[1] != 'Q') {
                    // Regular camera frame (6 bytes)
                    UartProcessCameraFrame(handler);
                    handler->camera_frame_pos = 0;
                }
            }
        }
    } else if (handler->is_debug_uart) {
        // Text-based command processing for UART2 (Debug)
        for (uint16_t i = 0; i < len; i++) {
            char c = handler->rxProcessBuffer[i];
            if (c == '\r' || c == '\n') { // End of command
                if (handler->rxLineSize > 0) {
                    handler->rxLineBuffer[handler->rxLineSize] = '\0'; // Null-terminate
                    handler->lineReady = true; // Mark line as ready for processing
                    handler->rxLineSize = 0; // Reset for next line
                }
            } else {
                if (handler->rxLineSize < (UART_RX_BUFFER_SIZE - 1)) {
                    handler->rxLineBuffer[handler->rxLineSize++] = c;
                }
            }
        }
    }
    memset(handler->rxProcessBuffer, 0, len);
}

// Duoc goi khi co ngat UART IDLE line
void UartIdleCallback(UartHandler *handler) {
    __HAL_UART_CLEAR_IDLEFLAG(handler->huart);
    UartCheckDMA(handler);
}

// Calculate 16-bit checksum
uint16_t CalculateChecksum(uint8_t *data, uint16_t length) {
    uint16_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}

// Validate camera data range
bool ValidateCameraData(float x, float y) {
    const float MAX_COORDINATE = 9.5f;  // ±9.5cm range
    return (x >= -MAX_COORDINATE && x <= MAX_COORDINATE &&
            y >= -MAX_COORDINATE && y <= MAX_COORDINATE);
}

// Process camera binary frame
bool UartProcessCameraFrame(UartHandler *handler) {
    if (!handler->is_camera_uart) return false;

    uint8_t *frame = handler->camera_frame_buffer;

    // Verify frame structure: @<x_int8><y_int8>X<checksum_2bytes>
    if (frame[0] != CAMERA_FRAME_START || frame[3] != CAMERA_FRAME_STOP) {
        return false;
    }

    // Calculate checksum từ START đến STOP (4 bytes đầu)
    uint16_t calculated_checksum = CalculateChecksum(frame, 4);
    uint16_t received_checksum = frame[4] | ((uint16_t)frame[5] << 8); // Little endian
    
    if (calculated_checksum != received_checksum) {
        // Checksum mismatch - frame corrupted
        return false;
    }

    // Extract coordinates (int8 format)
    int8_t x_int = (int8_t)frame[1];
    int8_t y_int = (int8_t)frame[2];
    
    // Convert to float (divide by scale factor)
    float x = (float)x_int / COORDINATE_SCALE;
    float y = (float)y_int / COORDINATE_SCALE;

    // Validate range
    if (!ValidateCameraData(x, y)) {
        return false;
    }

    // Store in buffer
    CameraDataBuffer *buffer = &handler->camera_buffer;
    buffer->data[buffer->head].x = x;
    buffer->data[buffer->head].y = y;
    buffer->data[buffer->head].valid = true;
    buffer->data[buffer->head].timestamp = HAL_GetTick();

    buffer->head = (buffer->head + 1) % CAMERA_BUFFER_SIZE;
    if (buffer->count < CAMERA_BUFFER_SIZE) {
        buffer->count++;
    }
    buffer->last_received = HAL_GetTick();
    buffer->timeout = false;

    return true;
}

// Get latest camera data
bool UartGetLatestCameraData(UartHandler *handler, CameraData *data) {
    if (!handler->is_camera_uart || handler->camera_buffer.count == 0) {
        return false;
    }

    // Get most recent data
    uint8_t latest_index = (handler->camera_buffer.head - 1 + CAMERA_BUFFER_SIZE) % CAMERA_BUFFER_SIZE;
    *data = handler->camera_buffer.data[latest_index];
    return data->valid;
}

// Get average camera data from multiple samples
bool UartGetAverageCameraData(UartHandler *handler, CameraData *data, uint8_t samples) {
    if (!handler->is_camera_uart || handler->camera_buffer.count == 0) {
        return false;
    }

    uint8_t count = (samples > handler->camera_buffer.count) ? handler->camera_buffer.count : samples;
    float sum_x = 0, sum_y = 0;
    uint32_t latest_timestamp = 0;

    for (uint8_t i = 0; i < count; i++) {
        uint8_t index = (handler->camera_buffer.head - 1 - i + CAMERA_BUFFER_SIZE) % CAMERA_BUFFER_SIZE;
        sum_x += handler->camera_buffer.data[index].x;
        sum_y += handler->camera_buffer.data[index].y;
        if (handler->camera_buffer.data[index].timestamp > latest_timestamp) {
            latest_timestamp = handler->camera_buffer.data[index].timestamp;
        }
    }

    data->x = sum_x / count;
    data->y = sum_y / count;
    data->valid = true;
    data->timestamp = latest_timestamp;

    return true;
}

// Check camera timeout
void UartCameraTimeout(UartHandler *handler, uint32_t timeout_ms) {
    if (!handler->is_camera_uart) return;

    uint32_t current_time = HAL_GetTick();
    if (current_time - handler->camera_buffer.last_received > timeout_ms) {
        handler->camera_buffer.timeout = true;
        // Clear buffer on timeout
        handler->camera_buffer.count = 0;
        handler->camera_buffer.head = 0;
    }
}


