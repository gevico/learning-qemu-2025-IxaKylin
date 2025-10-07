#include "crt.h"

static void custom_sort(uintptr_t addr, int array_num, int sort_num)
{
    asm volatile (
       ".insn r 0x7b, 6, 22, %0, %1, %2"
        : :"r"(sort_num), "r"(addr), "r"(array_num));
}

void bubble_sort(uint32_t arr[], int n) {
    for (int i = 0; i < n - 1; i++) {
        int swapped = 0;
        for (int j = 0; j < n - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                int temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
                swapped = 1;
            }
        }
        if (!swapped) {
            break;
        }
    }
}

void print_array(uint32_t arr[], int n) {
    for (int i = 0; i < n; i++) {
        printf("%d ", arr[i]);
    }
    printf("\n");
}

static void compare(uint32_t arr1[], uint32_t arr2[], int n)
{
    for (int i = 0; i < n; i++) {
        crt_assert(arr1[i] == arr2[i]);
    }
    printf("compare sort suceessful!\n");
}

static void test_sort(void)
{
    int arr1[32] = {3, 7, 23, 9, 81, 33, 04, 607747, 13, 2451, 323, 831, 0};
    int arr2[32] = {3, 7, 23, 9, 81, 33, 04, 607747, 13, 2451, 323, 831, 0};
    print_array(arr1, 32);
    bubble_sort(arr1, 16);
    custom_sort((uintptr_t)arr2, 32, 16);
    compare(arr1, arr2, 16);
}

int main(void)
{
    printf("Hello, RISC-V G233 Board\n");
    test_sort();
    return 0;
}
