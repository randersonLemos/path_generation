Creating share library on UNIX systems
gcc -c -fpic -g *.c
gcc -c -fpic -g *.cpp
gcc -shared -Wl,-soname,libransac.so -o libransac.so *.o -lc
gcc -shared ols.pic.o -L/usr/local/lib -lc -lgsl -lgslcblas -lm -o libols.so
gcc -Wall -fPIC -O -g abc1.c -c -o abc1.pic.o
