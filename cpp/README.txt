Creating share library on UNIX systems
-> gcc -c -fpic -g *.c
-> gcc -shared -Wl,-soname,libransac.so -o libransac.so *.o -lc
