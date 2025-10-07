#include "crt.h"

static void custom_crush(uintptr_t src, uintptr_t dst, int num)
{
    asm volatile (
       ".insn r 0x7b, 6, 38, %0, %1, %2"
        : :"r"(dst), "r"(src), "r"(num));
}

void pack_low4bits(const uint8_t *src, size_t src_len, uint8_t *dst, size_t *dst_len)
{
    size_t i = 0;
    size_t j = 0;

    while (i + 1 < src_len) {
        dst[j] = (src[i] & 0x0F) | ((src[i + 1] & 0x0F) << 4);
        i += 2;
        j++;
    }

    if (i < src_len) {
        dst[j] = src[i] & 0x0F;
        j++;
    }

    *dst_len = j;
}

static void compare(uint8_t arr1[], uint8_t arr2[], int n)
{
    for (int i = 0; i < n; i++) {
        crt_assert(arr1[i] == arr2[i]);
    }
    printf("compare crush suceessful!\n");
}

void print_array(uint8_t arr[], int n) {
    for (int i = 0; i < n; i++) {
        printf("%x ", arr[i]);
    }
    printf("\n");
}
int main(void)
{
    printf("Hello, RISC-V G233 Board\n");
    uint8_t src[] = {0xA, 0xB, 0xC, 0xD, 0xE, 0xF, 0x1, 0x2, 0x3, 0x4};
    size_t src_len = sizeof(src) / sizeof(src[0]);
    uint8_t dst1[(src_len + 1) / 2];
    uint8_t dst2[(src_len + 1) / 2];
    size_t dst_len;

    pack_low4bits(src, src_len, dst1, &dst_len);
    custom_crush((uintptr_t)src, (uintptr_t)dst2, src_len);
    compare(dst1, dst2, dst_len);

    return 0;
}
