#ifndef SKIDSTEER_EKF_H
#define SKIDSTEER_EKF_H

#include <math.h>

// EKF para robot skid-steering:
// x = [px, py, th, v, w, a]
// z = [omegaR_avg, omegaL_avg, ax, ay, wz]

class SkidSteerEKF {
public:
    float x[6];
    float P[6][6];
    float Q[6][6];
    float R[5][5];
    float dt;

    float r;   // radio rueda
    float L;   // brazo lateral efectivo = a + b

    SkidSteerEKF(float dt_,
                 float r_,
                 float L_,
                 const float Q_[6][6],
                 const float R_[5][5],
                 const float x0[6],
                 const float P0[6][6])
    {
        dt = dt_;
        r  = r_;
        L  = L_;

        for (int i = 0; i < 6; ++i) {
            x[i] = x0[i];
            for (int j = 0; j < 6; ++j) {
                P[i][j] = P0[i][j];
                Q[i][j] = Q_[i][j];
                if (i < 5 && j < 5) {
                    R[i][j] = R_[i][j];
                }
            }
        }
    }

    void predict() {
        float px = x[0];
        float py = x[1];
        float th = x[2];
        float v  = x[3];
        float w  = x[4];
        float a  = x[5];

        float px_ = px + dt * v * cosf(th);
        float py_ = py + dt * v * sinf(th);
        float th_ = th + dt * w;
        float v_  = v + dt * a;
        float w_  = w;
        float a_  = a;

        float x_pred[6] = {px_, py_, th_, v_, w_, a_};

        float F[6][6] = {0};
        for (int i = 0; i < 6; ++i) F[i][i] = 1.0f;

        F[0][2] = -dt * v * sinf(th);
        F[0][3] =  dt * cosf(th);
        F[1][2] =  dt * v * cosf(th);
        F[1][3] =  dt * sinf(th);
        F[2][4] =  dt;
        F[3][5] =  dt;

        float FP[6][6] = {0};
        float Pn[6][6] = {0};

        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                for (int k = 0; k < 6; ++k)
                    FP[i][j] += F[i][k] * P[k][j];

        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                float s = Q[i][j];
                for (int k = 0; k < 6; ++k) {
                    s += FP[i][k] * F[j][k];
                }
                Pn[i][j] = s;
            }
        }

        x_pred[2] = atan2f(sinf(x_pred[2]), cosf(x_pred[2]));

        for (int i = 0; i < 6; ++i) x[i] = x_pred[i];
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                P[i][j] = Pn[i][j];
    }

    void update(const float z[5]) {
        float v = x[3];
        float w = x[4];
        float a = x[5];

        float z_hat[5];
        z_hat[0] = (v + L * w) / r;  // omegaR_avg
        z_hat[1] = (v - L * w) / r;  // omegaL_avg
        z_hat[2] = a;                // ax
        z_hat[3] = v * w;            // ay aprox
        z_hat[4] = w;                // wz

        float H[5][6] = {0};

        H[0][3] = 1.0f / r;
        H[0][4] = L / r;

        H[1][3] = 1.0f / r;
        H[1][4] = -L / r;

        H[2][5] = 1.0f;

        H[3][3] = w;
        H[3][4] = v;

        H[4][4] = 1.0f;

        float y[5];
        for (int i = 0; i < 5; ++i) {
            y[i] = z[i] - z_hat[i];
        }

        float HP[5][6] = {0};
        float S[5][5]  = {0};

        for (int i = 0; i < 5; ++i)
            for (int j = 0; j < 6; ++j)
                for (int k = 0; k < 6; ++k)
                    HP[i][j] += H[i][k] * P[k][j];

        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                float s = R[i][j];
                for (int k = 0; k < 6; ++k) {
                    s += HP[i][k] * H[j][k];
                }
                S[i][j] = s;
            }
        }

        float S_inv[5][5];
        invert5x5(S, S_inv);

        float PHt[6][5] = {0};
        float K[6][5]   = {0};

        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 5; ++j)
                for (int k = 0; k < 6; ++k)
                    PHt[i][j] += P[i][k] * H[j][k];

        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 5; ++j)
                for (int k = 0; k < 5; ++k)
                    K[i][j] += PHt[i][k] * S_inv[k][j];

        float x_new[6];
        for (int i = 0; i < 6; ++i) {
            float s = x[i];
            for (int j = 0; j < 5; ++j) {
                s += K[i][j] * y[j];
            }
            x_new[i] = s;
        }

        x_new[2] = atan2f(sinf(x_new[2]), cosf(x_new[2]));

        float I_KH[6][6] = {0};
        float Pn[6][6]   = {0};

        for (int i = 0; i < 6; ++i)
            I_KH[i][i] = 1.0f;

        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                for (int k = 0; k < 5; ++k)
                    I_KH[i][j] -= K[i][k] * H[k][j];

        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) {
                float s = 0.0f;
                for (int k = 0; k < 6; ++k)
                    s += I_KH[i][k] * P[k][j];
                Pn[i][j] = s;
            }

        for (int i = 0; i < 6; ++i) x[i] = x_new[i];
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                P[i][j] = Pn[i][j];
    }

private:
    bool invert5x5(const float A[5][5], float B[5][5]) {

        float aug[5][10];

        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                aug[i][j] = A[i][j];
                aug[i][j + 5] = (i == j) ? 1.0f : 0.0f;
            }
        }

        for (int col = 0; col < 5; ++col) {

            int pivot = col;
            float maxVal = fabsf(aug[col][col]);

            for (int row = col + 1; row < 5; ++row) {
                float v = fabsf(aug[row][col]);

                if (v > maxVal) {
                    maxVal = v;
                    pivot = row;
                }
            }

            // No utilizar una matriz identidad si falla la inversión
            if (!isfinite(maxVal) || maxVal < 1e-8f) {
                return false;
            }

            if (pivot != col) {
                for (int j = 0; j < 10; ++j) {
                    float tmp = aug[col][j];
                    aug[col][j] = aug[pivot][j];
                    aug[pivot][j] = tmp;
                }
            }

            float pivVal = aug[col][col];

            for (int j = 0; j < 10; ++j) {
                aug[col][j] /= pivVal;
            }

            for (int row = 0; row < 5; ++row) {

                if (row == col) continue;

                float factor = aug[row][col];

                for (int j = 0; j < 10; ++j) {
                    aug[row][j] -= factor * aug[col][j];
                }
            }
        }

        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {

                B[i][j] = aug[i][j + 5];

                if (!isfinite(B[i][j])) {
                    return false;
                }
            }
        }

        return true;
    }
};

#endif
