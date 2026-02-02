#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include "stm32f4xx_hal.h"
#include "ball_position.h"
#include <stdbool.h>

// Kich thuoc bo dem UART
#define UART_RX_BUFFER_SIZE 1024  // Increased from 512 for 60fps
#define UART_TX_BUFFER_SIZE 256    // Increased from 128 for 60fps

// Ky tu ket thuc dong
#define UART_CR '\r'
#define UART_LF '\n'

// Frame protocol constants (shared by Camera and legacy PSO)
#define FRAME_START_BIT 0x40    // '@' character (64 decimal)
#define FRAME_STOP_BIT 0x58     // 'X' character (88 decimal)



// Camera data frame structure
#define CAMERA_FRAME_START FRAME_START_BIT
#define CAMERA_FRAME_STOP  FRAME_STOP_BIT
#define CAMERA_FRAME_SIZE  6       // 1+2+1+2 bytes: @<x_int8><y_int8>X<checksum_2bytes>
#define TARGET_FRAME_SIZE  7       // 1+1+2+1+2 bytes: @T<x_int8><y_int8>X<checksum_2bytes>
#define COORDINATE_SCALE   10      // Same as camera app

// Trajectory control frame structures
#define TRAJECTORY_STOP_SIZE    5   // 1+1+1+2 bytes: @S X <checksum_2bytes>
#define TRAJECTORY_CIRCLE_SIZE  9   // 1+1+2+2+1+2 bytes: @C<radius><speed>X<checksum_2bytes>
#define TRAJECTORY_SQUARE_SIZE  7   // 1+1+2+1+1 bytes: @Q<side_int16>X<checksum_2bytes>



// Max frame size across all supported frames (camera, target, trajectory)
#define CAMERA_MAX_FRAME_SIZE 9

typedef struct {
    float x;                   // X coordinate in cm
    float y;                   // Y coordinate in cm
    bool valid;               // Data validity flag
    uint32_t timestamp;       // Timestamp when received
} CameraData;

// Trajectory modes
typedef enum {
    TRAJECTORY_MODE_NONE = 0,
    TRAJECTORY_MODE_CIRCLE = 1
} TrajectoryMode;

typedef struct {
    TrajectoryMode mode;       // Current trajectory mode
    bool active;               // Is trajectory currently active
    uint32_t start_time;       // Start time of trajectory (ms)
    
    // Trajectory parameters
    float radius_cm;           // Circle radius (cm)
    float speed_rad_s;         // Circle angular speed (rad/s)
    
    // Current target position
    float target_x;            // Current target X position
    float target_y;            // Current target Y position
} TrajectoryControl;

// Camera data buffer for multiple values
#define CAMERA_BUFFER_SIZE 20  // Increased from 5 to 20 for 60fps
typedef struct {
    CameraData data[CAMERA_BUFFER_SIZE];
    uint8_t head;             // Write index
    uint8_t count;            // Number of valid entries
    uint32_t last_received;   // Last receive timestamp
    bool timeout;             // Timeout flag
} CameraDataBuffer;

// Cau truc UART Handler
typedef struct {
    UART_HandleTypeDef *huart;       // UART handle
    DMA_HandleTypeDef *hdma_rx;      // DMA handle RX

    uint8_t rxBuffer[UART_RX_BUFFER_SIZE];          // Vong DMA buffer
    uint8_t rxProcessBuffer[UART_RX_BUFFER_SIZE];   // Dem xu ly
    uint8_t rxLineBuffer[UART_RX_BUFFER_SIZE];      // Dem xu ly dong
    uint16_t rxLineSize;         // Do dai dong nhan duoc
    uint16_t oldPos;             // Vi tri cu trong buffer DMA
    uint16_t newPos;             // Vi tri moi
    bool lineReady;              // Co dong hoan chinh chua

    uint8_t txBuffer[UART_TX_BUFFER_SIZE]; // Bo dem truyen
    bool txBusy;                 // Co dang truyen khong

    // Camera specific data (for UART1)
    CameraDataBuffer camera_buffer;
    uint8_t camera_frame_buffer[CAMERA_MAX_FRAME_SIZE];  // Max size across all frames (up to 9 bytes)
    uint8_t camera_frame_pos;
    bool is_camera_uart;         // Flag to identify camera UART
    
    // Target frame priority processing
    bool target_frame_ready;     // Flag indicating target frame is ready for processing
    
    // Trajectory control
    TrajectoryControl trajectory; // Trajectory control data
    bool trajectory_frame_ready;  // Unused (trajectory removed)
    
    // Debug specific data (for UART2) 
    bool is_debug_uart;          // Flag to identify debug UART
    

} UartHandler;

// Nguyen mau ham
void UartInit(UartHandler *handler, UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_rx);

void UartStartReceive(UartHandler *handler);
bool UartProcessData(UartHandler *handler, BallPosition *ball);

void UartTransmit(UartHandler *handler, const char *data, uint16_t size);
void UartTransmitString(UartHandler *handler, const char *str);
void UartDmaRxCpltCallback(UartHandler *handler);
void UartDmaTxCpltCallback(UartHandler *handler);
void UartIdleCallback(UartHandler *handler);
void UartProcessBuffer(UartHandler *handler, uint16_t len);
bool UartCheckDMA(UartHandler *handler);

// New function prototypes for camera data
bool UartProcessCameraFrame(UartHandler *handler);
bool UartGetLatestCameraData(UartHandler *handler, CameraData *data);
bool UartGetAverageCameraData(UartHandler *handler, CameraData *data, uint8_t samples);
void UartCameraTimeout(UartHandler *handler, uint32_t timeout_ms);
uint16_t CalculateChecksum(uint8_t *data, uint16_t length);
bool ValidateCameraData(float x, float y);

// Target coordinate processing
bool UartProcessTargetFrame(UartHandler *handler, float *target_x, float *target_y);

// Trajectory control processing
bool UartProcessTrajectoryFrame(UartHandler *handler);
void UartUpdateTrajectory(UartHandler *handler);
void UartStopTrajectory(UartHandler *handler);





#endif /* UART_HANDLER_H */
