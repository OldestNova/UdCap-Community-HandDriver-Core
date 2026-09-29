#pragma once
#include "UdCapV1Core.h"

// Inverse of officialHandPose on the measured DOFs. VRChat's scalar protocol
// cannot express the other local twist axes; skeleton protocols retain them.
inline std::array<double, 28> anglesWithHandOffset(std::array<double, 28> a,
                                                  UdTarget hand, const HandRotation &o) {
    const double side = hand == UD_TARGET_LEFT_HAND ? -1 : 1;
    a[3] += o.thumbFinger.proximal.x / 1.2;
    a[2] -= side * o.thumbFinger.proximal.y / 0.3;
    a[20] += side * o.thumbFinger.proximal.z / 0.3;
    a[1] -= side * o.thumbFinger.intermediate.y;
    a[0] -= side * o.thumbFinger.distal.y;
    const FingerRotation fingers[]{o.indexFinger,o.middleFinger,o.ringFinger,o.littleFinger};
    for (int f = 0; f < 4; ++f) {
        a[6+4*f] += side * fingers[f].proximal.z;
        a[5+4*f] += side * fingers[f].intermediate.z;
        a[4+4*f] += side * fingers[f].distal.z;
    }
    a[7] -= side * o.indexFinger.proximal.y;
    a[11] += o.middleFinger.proximal.y;
    a[15] += side * o.ringFinger.proximal.y;
    a[19] += side * o.littleFinger.proximal.y;
    a[21] += o.indexFinger.proximal.x;
    a[22] += o.littleFinger.proximal.x;
    return a;
}
