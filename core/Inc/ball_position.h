#ifndef BALL_POSITION_H
#define BALL_POSITION_H

#include <stdint.h>
#include <stdbool.h>

// Cau truc luu vi tri cua bong (don vi cm, tinh tu tam dia)
typedef struct {
    float x;            // Vi tri X cua bong
    float y;            // Vi tri Y cua bong
    bool isDetected;    // Co phat hien bong hay khong
    uint32_t timestamp; // Thoi diem cap nhat (milisecond)
} BallPosition;

// Nguyen mau cac ham xu ly vi tri bong
void BallPositionInit(BallPosition *ball);
bool BallPositionUpdate(BallPosition *ball, float x, float y);

#endif /* BALL_POSITION_H */
