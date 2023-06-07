/*
Author: Nishant Sharma
Class: ECE4122
Last Date Modified: 11/20/2021
Description: 
The purpose of this file is to solve the 2D Steady
State Heat conduction in a thin plate problem, using 
iteration. It uses the Cuda library, and uses the gpu 
to do the computation.   

Running Procedure and Clarifications:- 
Getting a Pace ice gpu job with one gpu, 
setting up VNC, and running:
module load gcc/9.2.0 cuda/11.1
nvcc *.cu

Only the milliseconds is printed in the console without the units
as per the instructions and example on piazza.

References and links:-
https://docs.nvidia.com/cuda/cuda-c-programming-guide/index.html (Cuda Examples and guide)
https://stackoverflow.com/questions/7876624/timing-cuda-operations (Using cudaEvent for timing)
https://cpp.hotexamples.com/examples/-/-/cudaGetDeviceProperties/cpp-cudagetdeviceproperties-function-examples.html (cuda getDeviceProperties)

*/

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cuda_runtime.h>

using namespace std;

/**
 * The Heat Compute function get the average of surrounding 
 * elements in the sheet and setting them in the copy array. 
 * This array contains the new values of the interior points in the sheet
 * and they will be copied over to the original array in the next method (Jacobian)
 *
 * Input Arguments: Sheet and Copy arrays, numElements = (N+1)^2 (Total number of elements), and N (Interior row width)
 */
__global__ void heatCompute(double *sheet, double *copy, int numElements, int N) 
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    int rowWidth = numElements / (N + 2);
    if (tid <= (numElements)) {
        int i = tid % rowWidth;
        int j = tid / rowWidth;
        if (i > 0 && i <= N && j > 0 && j <= N)
        {
            copy[(N+2) * i + j] = 0.25 * (sheet[(N+2) * (i - 1) + (j)] + sheet[(N + 2) * (i + 1) + (j)] + sheet[(N + 2) * (i) + (j - 1)] + sheet[(N + 2) * (i) + (j + 1)]);
        }
    }
}

/**
 * Jacobian Function sets the original sheet variable, which 
 * contains the main 2D array to the values in copy, which are
 * the updated averages. 
 * 
 * Input Arguments: Sheet and Copy array, numElemetns: (N+2) * (N+2), N: Has the interior row
 */
__global__ void jacobian(double *sheet, double *copy, int numElements, int N) 
{
    int tid = blockDim.x * blockIdx.x + threadIdx.x;
    int rowWidth = numElements / (N + 2);
    if (tid <= (numElements)) {
        int i = tid % rowWidth;
        int j = tid / rowWidth;
        if (i > 0 && i <= N && j > 0 && j <= N)
        {
            sheet[(N + 2) * i + j] = copy[(N + 2) * i + j];
        }
    }
}

/**
 * Host main function, takes in the input arguments
 * for N and I, where N is the interior row width of the sheet, and 
 * I is the number of iterations to run the algorithm. Error checking 
 * is done, such that only positive values for -N and -I as passed. 
 * Finally the arrays are created copied over to the GPU, and the GPU 
 * routines are called. 
 */
int main(int argc, char * argv[])
{
    int N = 0;
    int I = 0;
    if (argc != 5)
    {
      cout << "Invalid parameters, please check your values." << endl;
      return 0;
    }
    if (strcmp("-N", argv[1]) != 0)
    {
      cout << "Invalid parameters, please check your values." << argv[1] << endl;
      return 0;
    }
    if (strcmp("-I", argv[3]) != 0)
    {
      cout << "Invalid parameters, please check your values." << endl;
      return 0;
    }
    try
    {
      N = atoi(argv[2]);
      I = atoi(argv[4]);
    }
    catch (exception & e)
    {
      cout << "Invalid parameters, please check your values." << endl;
      return 0;
    }
    if (N <= 0 || I <= 0)
    {
      cout << "Invalid parameters, please check your values." << endl;
      return 0;
    }

    // Creating the host sheet and copy array
    size_t size = (N + 2) * (N + 2) * sizeof(double);
    double *h_sheet = (double *)malloc(size);
    double *h_copy = (double *)malloc(size);
    int numElements = (N+2)*(N+2);

    // Initialization
    for (int i = 0; i < numElements; i++)
    {
        if ((i > 0.3 * (N + 2 - 1)) && (i < 0.7 * (N + 2 - 1)))
        {
            h_sheet[i] = 100.0;
        }
        else
        {
            h_sheet[i] = 20.0;
        }
        h_copy[i] = 0.0;
    }
    
    // Device sheet and copy array pointers created and data has been copied
    double *d_sheet = NULL;
    cudaMalloc(&d_sheet, size);
    
    double *d_copy = NULL;
    cudaMalloc(&d_copy, size);
    
    cudaMemcpy(d_sheet, h_sheet, size, cudaMemcpyHostToDevice);
    cudaMemcpy(d_copy, h_copy, size, cudaMemcpyHostToDevice);

    // Getting maximum number of threads and running setting 
    // thread and block number
    int dev = 0;
    cudaGetDevice(&dev);
    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, dev);
    if (prop.maxThreadsPerBlock <= 0)
    {
        cout << "Error in Running, invalid max number of threads" << endl;
    }
    int threadsPerBlock = prop.maxThreadsPerBlock;
    int blocksPerGrid = (numElements + threadsPerBlock - 1) / threadsPerBlock;    
    

    // Recording time
    float time;
    cudaEvent_t start, stop;

    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    cudaEventRecord(start, 0);

    for(int iteration = 0; iteration < I; iteration++)
    {
        // Does it synchronously - Updates all first, then set it equal
        heatCompute<<<blocksPerGrid, threadsPerBlock>>>(d_sheet, d_copy, numElements, N);
        jacobian<<<blocksPerGrid, threadsPerBlock>>>(d_sheet, d_copy, numElements, N);
    }

    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&time, start, stop);
    cout << fixed << setprecision(2) << time << endl;
    
    // Copy answer back from device to host
    cudaMemcpy(h_sheet, d_sheet, size, cudaMemcpyDeviceToHost);

    // Printing to csv
    ofstream outputFile;
    outputFile.open("finalTemperatures.csv", ios::out);
    outputFile << fixed << setprecision(6);
    for (int i = 0; i <= N+1; i++)
    {
        for (int j = 0; j <= N+1; j++)
        {
            outputFile << h_sheet[(N + 2) * i + j] << ",";
        }
        outputFile << "\n";
    }
    outputFile.close();

    cudaFree(d_sheet);
    cudaFree(d_copy);

    // Free host memory
    free(h_sheet);
    free(h_copy);
    return 0;
}
