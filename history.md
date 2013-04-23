# LPC11U/LPC13U Code Base - Core Revision History #

Major changes in the LPC11U/LPC13U code base by code base version number.

## 0.8.1 [In Progress] ##

- 'main' entry point moved to board-specific files ('src/boards/*')
- Removed main.c from src root
- LPCXpresso/Red Suite project files now default to the LPC1347
- Moved messages.c to drivers/rf/chibi
- Removed some unnecessary files
- Added binary.h to simplify binary access across toolchains (removed '0b' references)
- Added 'get_fattime' to board files (get timestamp for FAT32 and SD cards)

## 0.8.0 [2 April 2013] ##

- First public release
