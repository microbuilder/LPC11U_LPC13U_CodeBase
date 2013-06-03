# FIFO #

**fifo.c** attempts to provide a generic buffer that can be used as either a **classic FIFO** (first in, first out), or as a **circular buffer** (by making the fifo 'overwrittable').  Various type lengths can be assigned, so it can also be used with structs, or any arbitrary data type.

## Using fifo.c as a Classic FIFO Buffer ##

The following code will create a FIFO buffer named 'buffer' that is 8 int32\_t values long, with the 'overwrittable' flag set to false, meaning that when the buffer is full we won't be able to add new values via 'fifo\_write' until a previous value is read out.

The last argument is option, but in the case we will disable the USB interrupt while the FIFO is being manipulated to avoid any conflicts from incoming USB interrupts.  Simple assign NULL to this parameter if you don't want to enable the IRQ mutex support.
```
  FIFO_DEF(buffer, 8, int32_t, false, USB_IRQn);
```
To write an entry into the fifo, we call the 'fifo\_write' function, which will return either true or false depending on if the data was successfully written (for example if the fifo is already full):
```
  bool success;
  success = fifo_write(&buffer, 12345);
```
To read an entry from the fifo, we call the 'fifo\_read' or 'fifo\_readarray' functions:
```
  bool success;
  int32_t val;
  success = fifo_read(&buffer, &val);  // True if the buffer is not empty
```
We can also clear the fifo with the 'fifo\_clear' function:
```
  fifo_clear(&buffer);
```

## Using fifo.c as a Circular Buffer ##

The following code will create a circular buffer named 'cbuffer' using fifo.c, configuring it to be 8 samples wide, and using floating point data (meaning 32 bytes of memory will be used, since a single float takes 4 bytes, multiplied by our buffer size of 8 samples).  Notice the 'true' parameter, which means that this fifo instance is overwrittable, which is what allows us to emulate a circular buffer:
```
  // Create a circular buffer named 'cbuffer' 8 float values wide
  FIFO_DEF(cbuffer, 8, float, true, NULL);
```
To do the same for uint16\_t data, we would use the following code, which would require 16 bytes of memory since uint16\_t uses two bytes compared to four for float:
```
  // Create a circular buffer named 'cbuffer' 8 uint16_t values wide
  FIFO_DEF(cbuffer, 8, uint16_t, true, NULL);
```
To add samples into the circular buffer, we simple call the 'add' function as follows (assuming we are using float values):
```
  fifo_write(&cbuffer, 1.5F);
```
To read the samples back, we need to use the 'fifo\_peek' function (since fifo\_read is destructive and is used for a classic FIFO configuration):
```
  bool success;
  float val;
  // Assign item 1 in the circular buffer to 'val'
  success = fifo_peek(&cbuffer, 1, &val);   // Peek since read is destructive!
  // Assign item 3 in the circular buffer to 'val'
  success = fifo_peek(&cbuffer, 3, &val);   // Peek since read is destructive!
```
