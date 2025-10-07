#include "crt.h"

static void custom_dma(uintptr_t src, uintptr_t dst, int grain_size)
{
    asm volatile (
       ".insn r 0x7b, 6, 6, %0, %1, %2"
        : :"r"(dst), "r"(src), "r"(grain_size));
}

void print_matrix(uint32_t *mat, const char* name, int M, int N)
{
    printf("%s = \n", name);
    for (int i = 0; i < M; i++) {
        for (int j = 0; j < N; j++) {
            printf("%4d ", mat[i * N + j]);
        }
        printf("\n");
    }
    printf("\n");
}

void transpose(uint32_t *A, uint32_t *C, int M, int N) {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < M; j++) {
            C[i * M + j] = A[j * N + i];
        }
    }
}

void compare(uint32_t *A, uint32_t *C, int M, int N)
{
    for (int i = 0; i < M; i++) {
        for (int j = 0; j < N; j++) {
            crt_assert(C[i * N + j] == A[i * N + j]);
        }
    }
    printf("Grain: %dx%d, compare sucessful!\n", M, N);
}

#define GEN_TEST_DMA_GRAIN(M, N, grain)           \
static void test_dma_grain_##M##x##N(void)        \
{                                                 \
    uint32_t A[M * N];                            \
    uint32_t C[M * N];                            \
    uint32_t D[M * N];                            \
                                                  \
    for (int i = 0; i < M; i++) {                 \
        for (int j = 0; j < N; j++) {             \
            A[i * N + j] = i * N + j;             \
        }                                         \
    }                                             \
    print_matrix(A, "A", M, N);                   \
    transpose(A, C, M, N);                        \
    custom_dma((uintptr_t)A, (uintptr_t)D, grain);\
    compare(C, D, M, N);                          \
}

GEN_TEST_DMA_GRAIN(8, 8, 0)
GEN_TEST_DMA_GRAIN(16, 16, 1)
GEN_TEST_DMA_GRAIN(32, 32, 2)

int main(void)
{
    test_dma_grain_8x8();
    test_dma_grain_16x16();
    test_dma_grain_32x32();
    return 0;
}
