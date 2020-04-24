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

    start = clock();
    for (int trial = 0; trial < times; ++trial) {
        // perform 'times' times
        for (int i = 0; i < 256; ++i) {
            for (int j = 0; j < 256; ++j) {
                /* non-blocked transpose */
                output[j][i] = input[i][j];
            }
        }
    }
    end = clock();
    fprintf(stdout, "average time taken: %g seconds\t(unblocked)\n", ((double)(end - start) / (CLOCKS_PER_SEC * times)));
    return 0;
}

