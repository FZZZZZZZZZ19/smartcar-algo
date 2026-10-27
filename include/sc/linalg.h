/* 小型线性代数：矩阵乘法、求逆、离散 LQR
 *
 * 只支持 8x8 以内，够 LQR（2 状态路径跟踪、4 状态平衡车）用。
 * 嵌入式上部署时，K 可以离线算好后写死，运行时不需要这些函数。
 */
#ifndef SC_LINALG_H
#define SC_LINALG_H

#include "sc/common.h"

#define SC_LIN_MAX 8

typedef struct {
  int rows;
  int cols;
  float a[SC_LIN_MAX][SC_LIN_MAX];
} sc_mat_t;

void sc_mat_zero(sc_mat_t *m, int rows, int cols);
void sc_mat_eye(sc_mat_t *m, int n);
void sc_mat_copy(sc_mat_t *dst, const sc_mat_t *src);
void sc_mat_scale(sc_mat_t *m, float k);

int sc_mat_add(const sc_mat_t *A, const sc_mat_t *B, sc_mat_t *C);
int sc_mat_sub(const sc_mat_t *A, const sc_mat_t *B, sc_mat_t *C);
int sc_mat_mul(const sc_mat_t *A, const sc_mat_t *B, sc_mat_t *C);
void sc_mat_transpose(const sc_mat_t *A, sc_mat_t *At);

/* 高斯-约当求逆，成功返回 0 */
int sc_mat_inv(const sc_mat_t *A, sc_mat_t *inv);

/* 最大元素差的绝对值，用于迭代收敛判断 */
float sc_mat_max_diff(const sc_mat_t *A, const sc_mat_t *B);

/* 离散代数 Riccati 方程迭代求解，得到最优反馈增益 K（m x n）
 *   A: n x n, B: n x m, Q: n x n, R: m x m
 *   最小化 J = Σ (x'Qx + u'Ru)，u = -Kx
 * 返回迭代次数；<0 表示失败。 */
int sc_lqr_discrete(const sc_mat_t *A, const sc_mat_t *B, const sc_mat_t *Q,
                    const sc_mat_t *R, sc_mat_t *K, int max_iter, float tol);

/* 打印矩阵（调试用，格式到 stdout） */
void sc_mat_print(const char *name, const sc_mat_t *m);

#endif
