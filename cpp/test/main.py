import ctypes as ct

DOUBLE  = ct.c_float 
PDOUBLE = ct.POINTER(DOUBLE)
PPDOUBLE = ct.POINTER(PDOUBLE)

# An array of double* can be passed to a function as double**
DBL3ARR = DOUBLE * 3
PDBL3ARR = PDOUBLE * 3

ptr = PDBL3ARR()
for i in range(3):
  # fill out each pointer with an array of doubles
  ptr[i] = DBL3ARR()
  for j in range(3):
    ptr[i][j] = i+j


libtestFunctions = ct.CDLL("./libtestFunctions.so")
changeMatrix = getattr(libtestFunctions,'changeMatrix')

changeMatrix(ptr)
