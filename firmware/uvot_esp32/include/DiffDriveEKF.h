#ifndef DIFFDRIVE_EKF_H
#define DIFFDRIVE_EKF_H

#include <math.h>

// EKF para robot diferencial:
// x = [px, py, th, v, w, a]
// z = [omegaR, omegaL, ax, ay, wz]

class DiffDriveEKF {
public:
    float x[6];     // estado
    float P[6][6];  // covarianza
    float Q[6][6];  // ruido de proceso
    float R[5][5];  // ruido de medida
    float dt;       // periodo de muestreo

    float r;        // radio rueda
    float b;        // distancia entre ruedas

    DiffDriveEKF(float dt_,
                 float r_,
                 float b_,
                 const float Q_[6][6],
                 const float R_[5][5],
                 const float x0[6],
                 const float P0[6][6])
    {
        dt = dt_;
        r  = r_;
        b  = b_;

        // Copiar matrices
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

    // ---------------- PREDICCIÓN ----------------
    void predict() {
        float px = x[0];
        float py = x[1];
        float th = x[2];
        float v  = x[3];
        float w  = x[4];
        float a  = x[5];

        // Modelo de estado
        float px_ = px + dt * v * cosf(th);
        float py_ = py + dt * v * sinf(th);
        float th_ = th + dt * w;
        float v_  = v + dt * a;
        float w_  = w;
        float a_  = a;

        float x_pred[6] = {px_, py_, th_, v_, w_, a_};

        // Jacobiano F
        float F[6][6] = {0};
        for (int i = 0; i < 6; ++i) F[i][i] = 1.0f;

        F[0][2] = -dt * v * sinf(th);
        F[0][3] =  dt * cosf(th);
        F[1][2] =  dt * v * cosf(th);
        F[1][3] =  dt * sinf(th);
        F[2][4] =  dt;
        F[3][5] =  dt;

        // Pn = F P Fᵀ + Q
        float FP[6][6] = {0};
        float Pn[6][6] = {0};

        // FP = F*P
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                for (int k = 0; k < 6; ++k)
                    FP[i][j] += F[i][k] * P[k][j];

        // Pn = FP * Fᵀ + Q
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                float s = Q[i][j];
                for (int k = 0; k < 6; ++k) {
                    s += FP[i][k] * F[j][k];   // Fᵀ[k][j] = F[j][k]
                }
                Pn[i][j] = s;
            }
        }

        // Normalizar ángulo
        x_pred[2] = atan2f(sinf(x_pred[2]), cosf(x_pred[2]));

        // Guardar
        for (int i = 0; i < 6; ++i) x[i] = x_pred[i];
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                P[i][j] = Pn[i][j];
    }

    // ---------------- ACTUALIZACIÓN ----------------
    // z = [omegaR, omegaL, ax, ay, wz]
    void update(const float z[5]) {
        float v = x[3];
        float w = x[4];
        float a = x[5];

        // Modelo de medida h(x)
        float z_hat[5];
        z_hat[0] = (v + 0.5f * b * w) / r;  // omegaR
        z_hat[1] = (v - 0.5f * b * w) / r;  // omegaL
        z_hat[2] = a;                       // ax
        z_hat[3] = v * w;                   // ay
        z_hat[4] = w;                       // wz

        // Jacobiano H (5x6)
        float H[5][6] = {0};

        // omegaR
        H[0][3] = 1.0f / r;
        H[0][4] = 0.5f * b / r;

        // omegaL
        H[1][3] = 1.0f / r;
        H[1][4] = -0.5f * b / r;

        // ax
        H[2][5] = 1.0f;

        // ay = v*w
        H[3][3] = w;
        H[3][4] = v;

        // wz
        H[4][4] = 1.0f;

        // Innovación y = z - h(x)
        float y[5];
        for (int i = 0; i < 5; ++i) {
            y[i] = z[i] - z_hat[i];
        }

        // S = H P Hᵀ + R
        float HP[5][6] = {0};
        float S[5][5]  = {0};

        // HP = H * P
        for (int i = 0; i < 5; ++i)
            for (int j = 0; j < 6; ++j)
                for (int k = 0; k < 6; ++k)
                    HP[i][j] += H[i][k] * P[k][j];

        // S = HP * Hᵀ + R
        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                float s = R[i][j];
                for (int k = 0; k < 6; ++k) {
                    s += HP[i][k] * H[j][k]; // Hᵀ[k][j] = H[j][k]
                }
                S[i][j] = s;
            }
        }

        // Invertir S (5x5)
        float S_inv[5][5];
        invert5x5(S, S_inv);

        // K = P Hᵀ S⁻¹
        float PHt[6][5] = {0};
        float K[6][5]   = {0};

        // PHt = P * Hᵀ
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 5; ++j)
                for (int k = 0; k < 6; ++k)
                    PHt[i][j] += P[i][k] * H[j][k];

        // K = PHt * S_inv
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 5; ++j)
                for (int k = 0; k < 5; ++k)
                    K[i][j] += PHt[i][k] * S_inv[k][j];


        // Actualización de estado: x = x + K*y
        float x_new[6];
        for (int i = 0; i < 6; ++i) {
            float s = x[i];
            for (int j = 0; j < 5; ++j) {
                s += K[i][j] * y[j];
            }
            x_new[i] = s;
        }

        // Normalizar ángulo
        x_new[2] = atan2f(sinf(x_new[2]), cosf(x_new[2]));

        // Actualización de covarianza: P = (I - K H) P
        float I_KH[6][6] = {0};
        float Pn[6][6]   = {0};

        // I_KH = I - K*H
        for (int i = 0; i < 6; ++i)
            I_KH[i][i] = 1.0f;

        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                for (int k = 0; k < 5; ++k)
                    I_KH[i][j] -= K[i][k] * H[k][j];

        // Pn = I_KH * P
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) {
                float s = 0.0f;
                for (int k = 0; k < 6; ++k)
                    s += I_KH[i][k] * P[k][j];
                Pn[i][j] = s;
            }

        // Guardar
        for (int i = 0; i < 6; ++i) x[i] = x_new[i];
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                P[i][j] = Pn[i][j];
    }

private:
    // Inversa 5x5 por Gauss–Jordan
    void invert5x5(const float A[5][5], float B[5][5]) {
        float aug[5][10];

        // Matriz aumentada [A | I]
        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                aug[i][j]     = A[i][j];
                aug[i][j + 5] = (i == j) ? 1.0f : 0.0f;
            }
        }

        // Gauss–Jordan
        for (int col = 0; col < 5; ++col) {
            // Buscar pivote
            int pivot = col;
            float maxVal = fabsf(aug[col][col]);
            for (int row = col + 1; row < 5; ++row) {
                float v = fabsf(aug[row][col]);
                if (v > maxVal) {
                    maxVal = v;
                    pivot = row;
                }
            }

            // Si pivote muy pequeño -> devolver identidad para no petar
            if (maxVal < 1e-8f) {
                for (int i = 0; i < 5; ++i)
                    for (int j = 0; j < 5; ++j)
                        B[i][j] = (i == j) ? 1.0f : 0.0f;
                return;
            }

            // Intercambiar filas si hace falta
            if (pivot != col) {
                for (int j = 0; j < 10; ++j) {
                    float tmp     = aug[col][j];
                    aug[col][j]   = aug[pivot][j];
                    aug[pivot][j] = tmp;
                }
            }

            // Normalizar fila pivote
            float pivVal = aug[col][col];
            for (int j = 0; j < 10; ++j) {
                aug[col][j] /= pivVal;
            }

            // Eliminar en las otras filas
            for (int row = 0; row < 5; ++row) {
                if (row == col) continue;
                float factor = aug[row][col];
                for (int j = 0; j < 10; ++j) {
                    aug[row][j] -= factor * aug[col][j];
                }
            }
        }

        // Extraer parte derecha como inversa
        for (int i = 0; i < 5; ++i)
            for (int j = 0; j < 5; ++j)
                B[i][j] = aug[i][j + 5];
    }
};

#endif
