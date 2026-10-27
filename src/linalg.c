#include "sc/linalg.h"

#include <stdio.h>

void sc_mat_zero(sc_mat_t *m, int rows, int cols) {
  memset(m, 0, sizeof(*m));
  m->rows = rows;
  m->cols = cols;
}

void sc_mat_eye(sc_mat_t *m, int n) {
  sc_mat_zero(m, n, n);
  for (int i = 0; i < n; i++) {
    m->a[i][i] = 1.0f;
  }
}

void sc_mat_copy(sc_mat_t *dst, const sc_mat_t *src) {
  memcpy(dst, src, sizeof(*src));
}

void sc_mat_scale(sc_mat_t *m, float k) {
  for (int i = 0; i < m->rows; i++) {
    for (int j = 0; j < m->cols; j++) {
      m->a[i][j] *= k;
    }
  }
}

int sc_mat_add(const sc_mat_t *A, const sc_mat_t *B, sc_mat_t *C) {
  if (A->rows != B->rows || A->cols != B->cols) {
    return -1;
  }
  sc_mat_zero(C, A->rows, A->cols);
  for (int i = 0; i < A->rows; i++) {
    for (int j = 0; j < A->cols; j++) {
      C->a[i][j] = A->a[i][j] + B->a[i][j];
    }
  }
  return 0;
}

int sc_mat_sub(const sc_mat_t *A, const sc_mat_t *B, sc_mat_t *C) {
  if (A->rows != B->rows || A->cols != B->cols) {
    return -1;
  }
  sc_mat_zero(C, A->rows, A->cols);
  for (int i = 0; i < A->rows; i++) {
    for (int j = 0; j < A->cols; j++) {
      C->a[i][j] = A->a[i][j] - B->a[i][j];
    }
  }
  return 0;
}

int sc_mat_mul(const sc_mat_t *A, const sc_mat_t *B, sc_mat_t *C) {
  if (A->cols != B->rows) {
    return -1;
  }

  sc_mat_t tmp;
  sc_mat_zero(&tmp, A->rows, B->cols);

  for (int i = 0; i < A->rows; i++) {
    for (int k = 0; k < A->cols; k++) {
      float aik = A->a[i][k];
      if (aik == 0.0f) {
        continue;
      }
      for (int j = 0; j < B->cols; j++) {
        tmp.a[i][j] += aik * B->a[k][j];
      }
    }
  }

  *C = tmp;
  return 0;
}

void sc_mat_transpose(const sc_mat_t *A, sc_mat_t *At) {
  sc_mat_t tmp;
  sc_mat_zero(&tmp, A->cols, A->rows);
  for (int i = 0; i < A->rows; i++) {
    for (int j = 0; j < A->cols; j++) {
      tmp.a[j][i] = A->a[i][j];
    }
  }
  *At = tmp;
}

int sc_mat_inv(const sc_mat_t *A, sc_mat_t *inv) {
  int n = A->rows;
  if (n != A->cols || n <= 0 || n > SC_LIN_MAX) {
    return -1;
  }

  float aug[SC_LIN_MAX][2 * SC_LIN_MAX];
  memset(aug, 0, sizeof(aug));

  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      aug[i][j] = A->a[i][j];
    }
    aug[i][i + n] = 1.0f;
  }

  for (int col = 0; col < n; col++) {
    /* 部分主元，避免除以接近 0 的数 */
    int pivot = col;
    float best = fabsf(aug[col][col]);
    for (int r = col + 1; r < n; r++) {
      float v = fabsf(aug[r][col]);
      if (v > best) {
        best = v;
        pivot = r;
      }
    }

    if (best < 1e-9f) {
      return -1;
    }

    if (pivot != col) {
      for (int j = 0; j < 2 * n; j++) {
        float t = aug[col][j];
        aug[col][j] = aug[pivot][j];
        aug[pivot][j] = t;
      }
    }

    float inv_p = 1.0f / aug[col][col];
    for (int j = 0; j < 2 * n; j++) {
      aug[col][j] *= inv_p;
    }

    for (int r = 0; r < n; r++) {
      if (r == col) {
        continue;
      }
      float factor = aug[r][col];
      if (factor == 0.0f) {
        continue;
      }
      for (int j = 0; j < 2 * n; j++) {
        aug[r][j] -= factor * aug[col][j];
      }
    }
  }

  sc_mat_zero(inv, n, n);
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      inv->a[i][j] = aug[i][j + n];
    }
  }
  return 0;
}

float sc_mat_max_diff(const sc_mat_t *A, const sc_mat_t *B) {
  if (A->rows != B->rows || A->cols != B->cols) {
    return 1e9f;
  }
  float m = 0.0f;
  for (int i = 0; i < A->rows; i++) {
    for (int j = 0; j < A->cols; j++) {
      float d = fabsf(A->a[i][j] - B->a[i][j]);
      if (d > m) {
        m = d;
      }
    }
  }
  return m;
}

int sc_lqr_discrete(const sc_mat_t *A, const sc_mat_t *B, const sc_mat_t *Q,
                    const sc_mat_t *R, sc_mat_t *K, int max_iter, float tol) {
  int n = A->rows;
  int m = B->cols;

  if (A->cols != n || B->rows != n) {
    return -1;
  }
  if (Q->rows != n || Q->cols != n || R->rows != m || R->cols != m) {
    return -1;
  }

  sc_mat_t P;
  sc_mat_copy(&P, Q);

  sc_mat_t At, Bt;
  sc_mat_transpose(A, &At);
  sc_mat_transpose(B, &Bt);

  if (max_iter <= 0) {
    max_iter = 200;
  }
  if (tol <= 0.0f) {
    tol = 1e-6f;
  }

  sc_mat_t K_cur;
  sc_mat_zero(&K_cur, m, n);

  for (int iter = 0; iter < max_iter; iter++) {
    /* S = R + B' P B */
    sc_mat_t BtP, BtPB, S, Sinv;
    if (sc_mat_mul(&Bt, &P, &BtP) != 0) {
      return -1;
    }
    if (sc_mat_mul(&BtP, B, &BtPB) != 0) {
      return -1;
    }
    if (sc_mat_add(R, &BtPB, &S) != 0) {
      return -1;
    }
    if (sc_mat_inv(&S, &Sinv) != 0) {
      return -1;
    }

    /* K = S^-1 B' P A */
    sc_mat_t BtPA;
    if (sc_mat_mul(&BtP, A, &BtPA) != 0) {
      return -1;
    }
    sc_mat_t K_new;
    if (sc_mat_mul(&Sinv, &BtPA, &K_new) != 0) {
      return -1;
    }

    /* P_new = A' P A - A' P B K + Q */
    sc_mat_t PA, AtPA, PB, PBK, AtPBK, P_new, tmp;
    if (sc_mat_mul(&P, A, &PA) != 0) {
      return -1;
    }
    if (sc_mat_mul(&At, &PA, &AtPA) != 0) {
      return -1;
    }
    if (sc_mat_mul(&P, B, &PB) != 0) {
      return -1;
    }
    if (sc_mat_mul(&PB, &K_new, &PBK) != 0) {
      return -1;
    }
    if (sc_mat_mul(&At, &PBK, &AtPBK) != 0) {
      return -1;
    }
    if (sc_mat_sub(&AtPA, &AtPBK, &tmp) != 0) {
      return -1;
    }
    if (sc_mat_add(&tmp, Q, &P_new) != 0) {
      return -1;
    }

    float diff = sc_mat_max_diff(&P_new, &P);
    P = P_new;
    K_cur = K_new;

    if (diff < tol) {
      *K = K_cur;
      return iter + 1;
    }
  }

  *K = K_cur;
  return max_iter;
}

void sc_mat_print(const char *name, const sc_mat_t *m) {
  printf("%s (%dx%d):\n", name, m->rows, m->cols);
  for (int i = 0; i < m->rows; i++) {
    printf("  [");
    for (int j = 0; j < m->cols; j++) {
      printf(" %9.4f", (double)m->a[i][j]);
    }
    printf(" ]\n");
  }
}
