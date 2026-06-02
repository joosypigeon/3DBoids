#!/usr/bin/env bash

set -e

echo "Installing OpenMP support for Debian/Ubuntu..."

# Check that apt exists
if ! command -v apt >/dev/null 2>&1; then
    echo "Error: this script is intended for Debian/Ubuntu-based systems using apt."
    exit 1
fi

# Update package lists
sudo apt update

# Install GCC, G++, Clang, Make, CMake, and OpenMP runtimes
sudo apt install -y \
    build-essential \
    gcc \
    g++ \
    clang \
    make \
    cmake \
    libgomp1 \
    libomp-dev

echo
echo "OpenMP installation complete."
echo

echo "Checking GCC OpenMP support..."
cat > /tmp/openmp_test.c << 'EOF'
#include <stdio.h>
#include <omp.h>

int main(void)
{
    #pragma omp parallel
    {
        int id = omp_get_thread_num();
        int total = omp_get_num_threads();

        #pragma omp critical
        {
            printf("Hello from thread %d of %d\n", id, total);
        }
    }

    return 0;
}
EOF

gcc -fopenmp /tmp/openmp_test.c -o /tmp/openmp_test

echo
echo "Running OpenMP test:"
/tmp/openmp_test

echo
echo "If you see messages from several threads, OpenMP is working."
