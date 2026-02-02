#include "robot_geometry.h"

// Tinh chieu cao toi da dua vao cau truc RobotGeometry
float ComputeMaxH(const RobotGeometry *geom) {
    return sqrtf(SQUARE(geom->l1 + geom->l2) - SQUARE(geom->lp - geom->lb));
}

// Tinh chieu cao toi thieu - MATCH PYTHON PROJECT EXACTLY
float ComputeMinH(const RobotGeometry *geom) {
    // Python logic:
    // if self.l1 > self.l2:
    //     return math.sqrt((self.l1 ** 2) - ((self.lb + self.l2 - self.lp) ** 2))
    // elif self.l2 > self.l1:
    //     return math.sqrt(((self.l2 - self.l1) ** 2) - ((self.lp - self.lb) ** 2))
    // else:
    //     return 0
    
    if (geom->l1 > geom->l2) {
        float inner = SQUARE(geom->l1) - SQUARE(geom->lb + geom->l2 - geom->lp);
        if (inner <= 0.0f) {
            return 0.0f;  // Match Python fallback
        }
        return sqrtf(inner);
    } else if (geom->l2 > geom->l1) {
        float inner = SQUARE(geom->l2 - geom->l1) - SQUARE(geom->lp - geom->lb);
        if (inner <= 0.0f) {
            return 0.0f;  // Match Python fallback
        }
        return sqrtf(inner);
    } else {
        return 0.0f; // Match Python: return 0
    }
}
