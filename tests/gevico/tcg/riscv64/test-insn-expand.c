#include "crt.h"

static void custom_expand(uintptr_t src, uintptr_t dst, int num)
{
    asm volatile (
       ".insn r 0x7b, 6, 54, %0, %1, %2"
        : :"r"(dst), "r"(src), "r"(num));
}

void split_to_4bits(const uint8_t *src, size_t src_len, uint8_t *dst, size_t *dst_len)
{
    size_t j = 0;
    for (size_t i = 0; i < src_len; i++) {
        dst[j++] = src[i] & 0x0F;
        dst[j++] = (src[i] >> 4) & 0x0F;
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
    uint8_t src[] = {0xAB, 0xBC, 0xCD, 0xDE, 0xEF, 0xFA, 0x13, 0x24, 0x63, 0x74};
    size_t src_len = sizeof(src) / sizeof(src[0]);
    uint8_t dst1[src_len * 2];
    uint8_t dst2[src_len * 2];
    size_t dst_len;

    split_to_4bits(src, src_len, dst1, &dst_len);
    custom_expand((uintptr_t)src, (uintptr_t)dst2, src_len);
    compare(dst1, dst2, dst_len);

    return 0;
}
