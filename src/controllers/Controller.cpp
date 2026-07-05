#include "Controller.h"
#include "common/Pose2d.h"
#include <algorithm>
#include <cmath>

Controller::Controller() {
}

void Controller::reset(const Pose2d &initialPose) {
}

Eigen::Vector<double, 2> Controller::calculate(const Eigen::Vector<double, 7> &x) {
    m_u = Eigen::Vector<double, 2>::Zero();

    return m_u;
}

Eigen::Vector<double, 7> Controller::dynamics(const Eigen::Vector<double, 7> &x, const Eigen::Vector<double, 2> &u) {
    Eigen::Vector<double, 7> xdot = Eigen::Vector<double, 7>::Zero();
    // TODO: Implement actual dynamics
    return xdot;
}

//it will be called by implicitly eigen function
inline float cosf(float x) {
    return std::cos(x);
}

//it will be called by implicitly eigen function
inline float sinf(float x) {
    return std::sin(x);
}

//how to replace these to eigen function
static void matMultiply(const float *_matrix1, const float *_matrix2, float *_matrixOut,
                        const int _m, const int _l, const int _n) {
    float tmp;
    int i, j, k;
    for (i = 0; i < _m; i++) {
        for (j = 0; j < _n; j++) {
            tmp = 0.0f;
            for (k = 0; k < _l; k++) {
                tmp += _matrix1[_l * i + k] * _matrix2[_n * k + j];
            }
            _matrixOut[_n * i + j] = tmp;
        }
    }
}

//how to replace these to eigen function
static void rotMatToEulerAngle(const float *_rotationM, float *_eulerAngles) {
    float A, B, C, cb;

    if (fabs(_rotationM[6]) >= 1.0 - 0.0001) {
        if (_rotationM[6] < 0) {
            A = 0.0f;
            B = (float) M_PI_2;
            C = atan2f(_rotationM[1], _rotationM[4]);
        } else {
            A = 0.0f;
            B = -(float) M_PI_2;
            C = -atan2f(_rotationM[1], _rotationM[4]);
        }
    } else {
        B = atan2f(-_rotationM[6], sqrtf(_rotationM[0] * _rotationM[0] + _rotationM[3] * _rotationM[3]));
        cb = cosf(B);
        if (fabs(cb) < 1e-6f) {
            A = 0.0f;
            C = 0.0f;
        } else {
            A = atan2f(_rotationM[3] / cb, _rotationM[0] / cb);
            C = atan2f(_rotationM[7] / cb, _rotationM[8] / cb);
        }
    }

    _eulerAngles[0] = C;
    _eulerAngles[1] = B;
    _eulerAngles[2] = A;
}

//how to replace these to eigen function
static void eulerAngleToRotMat(const float *_eulerAngles, float *_rotationM) {
    float ca, cb, cc, sa, sb, sc;

    cc = cosf(_eulerAngles[0]);
    cb = cosf(_eulerAngles[1]);
    ca = cosf(_eulerAngles[2]);
    sc = sinf(_eulerAngles[0]);
    sb = sinf(_eulerAngles[1]);
    sa = sinf(_eulerAngles[2]);

    _rotationM[0] = ca * cb;
    _rotationM[1] = ca * sb * sc - sa * cc;
    _rotationM[2] = ca * sb * cc + sa * sc;
    _rotationM[3] = sa * cb;
    _rotationM[4] = sa * sb * sc + ca * cc;
    _rotationM[5] = sa * sb * cc - ca * sc;
    _rotationM[6] = -sb;
    _rotationM[7] = cb * sc;
    _rotationM[8] = cb * cc;
}