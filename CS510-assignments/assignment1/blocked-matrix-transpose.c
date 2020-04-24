#include <stdio.h>
#include <time.h>
#include <stdlib.h>

int main(int argc, char *argv[]) {
    clock_t start, end;
    int times = 50;
    /* initialize input & output matrix */
    double **input = (double**)calloc(256, sizeof(double*));
    double **output = (double**)calloc(256, sizeof(double*));
    for (int k = 0; k < 256; ++k) {
        input[k] = (double*)calloc(256, sizeof(double));
        output[k] = (double*)calloc(256, sizeof(double));
    }
    /* sweep B from 1 to 256 */
    for (int B = 1; B <= 256; B*=2) {
        start = clock();
        for (int trial = 0; trial < times; ++trial) {
            // perform 'times' times
            for (int i = 0; i < 256; i+=B) {
                for (int j = 0; j < 256; j+=B) {
                    /* block-wise transpose */
                    for (int out_col = 0; out_col < B; ++out_col) {
                        for (int out_row = 0; out_row < B; ++out_row) {
                            output[j + out_row][i + out_col] = input[i + out_col][j + out_row];
                        }
                    }
                }
            }
        }
        end = clock();
        fprintf(stdout, "average time taken: %g seconds\t(B = %3d)\n", ((double)(end - start) / (CLOCKS_PER_SEC * times)), B);
    }
    return 0;
}

