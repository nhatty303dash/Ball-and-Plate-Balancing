#include "ball_position.h"
#include "stm32f4xx_hal.h"

// Khoi tao cau truc BallPosition
void BallPositionInit(BallPosition *ball) {
    ball->x = 0.0f;               // Dat vi tri x = 0
    ball->y = 0.0f;               // Dat vi tri y = 0
    ball->isDetected = false;     // Chua phat hien bong
    ball->timestamp = HAL_GetTick();  // Lay thoi gian hien tai
}

// Cap nhat vi tri bong moi
bool BallPositionUpdate(BallPosition *ball, float x, float y) {
    ball->x = x;                  // Cap nhat vi tri x
    ball->y = y;                  // Cap nhat vi tri y
    ball->isDetected = true;      // Danh dau da phat hien
    ball->timestamp = HAL_GetTick();  // Cap nhat timestamp
    return true;                  // Tra ve thanh cong
}
