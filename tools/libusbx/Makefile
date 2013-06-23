CC     = gcc
CFLAGS = -Wall -O2 -std=c99 -I. -L./mingw32
LIBS   = -lusb-1.0
DEPS   = libusb.h
OBJ    = bulktest.o

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(LIBS)

bulktest: $(OBJ)
	gcc -o $@ $^ $(CFLAGS) $(LIBS)
  
clean: 
	rm -f bulktest.exe
	rm -f $(OBJ)
