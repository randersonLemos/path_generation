Creating share library on UNIX systems
-> gcc -c -fpic -g *.c
-> gcc -c -fpic -g *.cpp
-> gcc -shared -Wl,-soname,libransac.so -o libransac.so *.o -lc
