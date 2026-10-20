#ifndef OMNIDRIVE_EKF_H
#define OMNIDRIVE_EKF_H

#include <math.h>

// EKF para robot omnidireccional:
// x = [px, py, th, vx_body, vy_body, wz, ax_body, ay_body]
// z = [w0, w1, w2, w3, wz_imu, ax_imu, ay_imu]

class OmniDriveEKF {
public:
    float x[8];
    float P[8][8];
    float Q[8][8];
    float R[7][7];
    float dt;

    float r;
    float a;
    float b;
    float L; // a + b

    OmniDriveEKF(float dt_,
                 float r_,
                 float a_,
                 float b_,
                 const float Q_[8][8],
                 const float R_[7][7],
                 const float x0[8],
                 const float P0[8][8])
    {
        dt = dt_;
        r  = r_;
        a  = a_;
        b  = b_;
        L  = a_ + b_;

        for (int i = 0; i < 8; ++i) {
            x[i] = x0[i];
            for (int j = 0; j < 8; ++j) {
                P[i][j] = P0[i][j];
                Q[i][j] = Q_[i][j];
                if (i < 7 && j < 7) {
                    R[i][j] = R_[i][j];
                }
            }
        }
    }

    void predict() {
        float px = x[0];
        float py = x[1];
        float th = x[2];
        float vx = x[3];
        float vy = x[4];
        float wz = x[5];
        float ax = x[6];
        float ay = x[7];

        float c = cosf(th);
        float s = sinf(th);

        float px_ = px + dt * (vx * c - vy * s);
        float py_ = py + dt * (vx * s + vy * c);
        float th_ = th + dt * wz;
        float vx_ = vx + dt * ax;
        float vy_ = vy + dt * ay;
        float wz_ = wz;
        float ax_ = ax;
        float ay_ = ay;

        float x_pred[8] = {px_, py_, th_, vx_, vy_, wz_, ax_, ay_};

        float F[8][8] = {0};
        for (int i = 0; i < 8; ++i) F[i][i] = 1.0f;

        F[0][2] = dt * (-vx * s - vy * c);
        F[0][3] = dt * c;
        F[0][4] = -dt * s;

        F[1][2] = dt * (vx * c - vy * s);
        F[1][3] = dt * s;
        F[1][4] = dt * c;

        F[2][5] = dt;
        F[3][6] = dt;
        F[4][7] = dt;

        float FP[8][8] = {0};
        float Pn[8][8] = {0};

        for (int i = 0; i < 8; ++i)
            for (int j = 0; j < 8; ++j)
                for (int k = 0; k < 8; ++k)
                    FP[i][j] += F[i][k] * P[k][j];

        for (int i = 0; i < 8; ++i) {
            for (int j = 0; j < 8; ++j) {
                float ssum = Q[i][j];
                for (int k = 0; k < 8; ++k) {
                    ssum += FP[i][k] * F[j][k];
                }
                Pn[i][j] = ssum;
            }
        }

        x_pred[2] = atan2f(sinf(x_pred[2]), cosf(x_pred[2]));

        for (int i = 0; i < 8; ++i) x[i] = x_pred[i];
        for (int i = 0; i < 8; ++i)
            for (int j = 0; j < 8; ++j)
                P[i][j] = Pn[i][j];
    }

    void update(const float z[7]) {
        float vx = x[3];
        float vy = x[4];
        float wz = x[5];
        float ax = x[6];
        float ay = x[7];

        // Modelo de ruedas omni:
        // w0 = (vx + vy - L*wz)/r
        // w1 = (vx - vy + L*wz)/r
        // w2 = (vx + vy + L*wz)/r
        // w3 = (vx - vy - L*wz)/r

        float z_hat[7];
        z_hat[0] = (vx + vy - L * wz) / r;
        z_hat[1] = (vx - vy + L * wz) / r;
        z_hat[2] = (vx + vy + L * wz) / r;
        z_hat[3] = (vx - vy - L * wz) / r;
        z_hat[4] = wz;
        z_hat[5] = ax;
        z_hat[6] = ay;

        float H[7][8] = {0};

        // w0
        H[0][3] =  1.0f / r;
        H[0][4] =  1.0f / r;
        H[0][5] = -L / r;

        // w1
        H[1][3] =  1.0f / r;
        H[1][4] = -1.0f / r;
        H[1][5] =  L / r;

        // w2
        H[2][3] =  1.0f / r;
        H[2][4] =  1.0f / r;
        H[2][5] =  L / r;

        // w3
        H[3][3] =  1.0f / r;
        H[3][4] = -1.0f / r;
        H[3][5] = -L / r;

        // wz
        H[4][5] = 1.0f;

        // ax
        H[5][6] = 1.0f;

        // ay
        H[6][7] = 1.0f;

        float y[7];
        for (int i = 0; i < 7; ++i) {
            y[i] = z[i] - z_hat[i];
        }

        float HP[7][8] = {0};
        float S[7][7]  = {0};

        for (int i = 0; i < 7; ++i)
            for (int j = 0; j < 8; ++j)
                for (int k = 0; k < 8; ++k)
                    HP[i][j] += H[i][k] * P[k][j];

        for (int i = 0; i < 7; ++i) {
            for (int j = 0; j < 7; ++j) {
                float ssum = R[i][j];
                for (int k = 0; k < 8; ++k) {
                    ssum += HP[i][k] * H[j][k];
                }
                S[i][j] = ssum;
            }
        }

        float S_inv[7][7];
        invert7x7(S, S_inv);

        float PHt[8][7] = {0};
        float K[8][7]   = {0};

        for (int i = 0; i < 8; ++i)
            for (int j = 0; j < 7; ++j)
                for (int k = 0; k < 8; ++k)
                    PHt[i][j] += P[i][k] * H[j][k];

        for (int i = 0; i < 8; ++i)
            for (int j = 0; j < 7; ++j)
                for (int k = 0; k < 7; ++k)
                    K[i][j] += PHt[i][k] * S_inv[k][j];

        float x_new[8];
        for (int i = 0; i < 8; ++i) {
            float ssum = x[i];
            for (int j = 0; j < 7; ++j) {
                ssum += K[i][j] * y[j];
            }
            x_new[i] = ssum;
        }

        x_new[2] = atan2f(sinf(x_new[2]), cosf(x_new[2]));

        float I_KH[8][8] = {0};
        float Pn[8][8]   = {0};

        for (int i = 0; i < 8; ++i)
            I_KH[i][i] = 1.0f;

        for (int i = 0; i < 8; ++i)
            for (int j = 0; j < 8; ++j)
                for (int k = 0; k < 7; ++k)
                    I_KH[i][j] -= K[i][k] * H[k][j];

        for (int i = 0; i < 8; ++i)
            for (int j = 0; j < 8; ++j) {
                float ssum = 0.0f;
                for (int k = 0; k < 8; ++k)
                    ssum += I_KH[i][k] * P[k][j];
                Pn[i][j] = ssum;
            }

        for (int i = 0; i < 8; ++i) x[i] = x_new[i];
        for (int i = 0; i < 8; ++i)
            for (int j = 0; j < 8; ++j)
                P[i][j] = Pn[i][j];
    }

private:
    void invert7x7(const float A[7][7], float B[7][7]) {
        float aug[7][14];

        for (int i = 0; i < 7; ++i) {
            for (int j = 0; j < 7; ++j) {
                aug[i][j]     = A[i][j];
                aug[i][j + 7] = (i == j) ? 1.0f : 0.0f;
            }
        }

        for (int col = 0; col < 7; ++col) {
            int pivot = col;
            float maxVal = fabsf(aug[col][col]);

            for (int row = col + 1; row < 7; ++row) {
                float v = fabsf(aug[row][col]);
                if (v > maxVal) {
                    maxVal = v;
                    pivot = row;
                }
            }

            if (maxVal < 1e-8f) {
                for (int i = 0; i < 7; ++i)
                    for (int j = 0; j < 7; ++j)
                        B[i][j] = (i == j) ? 1.0f : 0.0f;
                return;
            }

            if (pivot != col) {
                for (int j = 0; j < 14; ++j) {
                    float tmp     = aug[col][j];
                    aug[col][j]   = aug[pivot][j];
                    aug[pivot][j] = tmp;
                }
            }

            float pivVal = aug[col][col];
            for (int j = 0; j < 14; ++j) {
                aug[col][j] /= pivVal;
            }

            for (int row = 0; row < 7; ++row) {
                if (row == col) continue;
                float factor = aug[row][col];
                for (int j = 0; j < 14; ++j) {
                    aug[row][j] -= factor * aug[col][j];
                }
            }
        }

        for (int i = 0; i < 7; ++i)
            for (int j = 0; j < 7; ++j)
                B[i][j] = aug[i][j + 7];
    }
};

#endif
