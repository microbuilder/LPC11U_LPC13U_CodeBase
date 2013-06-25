CC     = gcc
CFLAGS = -Wall -O2 -std=c99 -I. -L./mingw32
LIBS   = -lusb-1.0
DEPS   = libusb.h

all: bulktest protocoltest

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(LIBS)

bulktest: bulktest.o
	gcc -o bulktest.exe $^ $(CFLAGS) $(LIBS)

protocoltest: protocoltest.o
	gcc -o protocoltest.exe $^ $(CFLAGS) $(LIBS)
  
clean: 
	rm -f bulktest.exe
	rm -f protocoltest.exe
	rm -f *.o
