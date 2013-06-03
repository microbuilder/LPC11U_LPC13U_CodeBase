CC     = gcc
CFLAGS = -Wall -O2 -std=c99 -I. -L./mingw32
LIBS   = -lusb-1.0
DEPS   = libusb.h
OBJ    = listdevs.o

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(LIBS)

listdevs: $(OBJ)
	gcc -o $@ $^ $(CFLAGS) $(LIBS)
  
clean: 
	rm -f listdevs.exe
	rm -f $(OBJ)
