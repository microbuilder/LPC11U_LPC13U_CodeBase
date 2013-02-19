# Localisation Support #

While this feature is very much in development, a basic effort has been made to support some form of localisation in the code base.  

Particularly in a European context, it's essential to be able to offer your products or projects in multiple languages, but localisation needs to be taken into account at the earliest part of the development cycle or it can be both error prone and time consuming to force it onto the system later.

The current solution of a key-based lookup table isn't entirely satisfactory or robust, but as a basic starting point it will hopefully develop into something a bit more mature in future releases. 

## How Does it Work? ##

There are two enums defined in **localisation.h**:

- **culture_t**: Defines the cultures supported by the system, for example **CULTURE\_EN** or **CULTURE\_FR**

- **localisedTextKeys_t**: Defines the 'keys' that are used when looking up specific values

Based on these two enums, there is also a simple macro defined in localisation.h named **STRING**, which provides a localised string from the current culture based on the supplied localisedTextKeys_t entry.

If you execute the following code:

```
printf("%s %s", STRING(LOCALISATION_TEXT_No_ACK_received), CFG_PRINTF_NEWLINE);
```

You should get something similar to the following output (depending on the culture you've selected):

```
No ACK received
```

## Selecting a Culture ##

To change the current culture, you simply need to call the **localisation\_SetCulture(culture\_t culture)** function with the appropriate culture code (as defined in culture\_t above):

```
localisation_SetCulture(CULTURE_FR);
```

The default culture is set in localisation.c via **\_localisation\_currentCulture**. To change the default culture, simply change the value assigned to this variable:

```
/* Set the default culture/language */
static volatile culture_t _localisation_currentCulture = CULTURE_FR;
```

## Adding Strings to the Localisation Database ##

There are two steps involved in adding new strings to the localisation database.

**Step One: Create a new localisedTextKeys_t entry in localisation.h**

Add a new key to the localisedTextKeys_t enum in localisation.h with a unique name, keeping in mind that the C standard (see 5.2.4.1) only guarantees 63 characters for unique names.

**Step Two: Add entries for the new key in the appropriate 'locale\_culture.dat' file**

Next open all of the locale_xx.dat files used by the system and add a new entry for your key, following the example below (for the locale_en.dat file):

```
LOCALE_EN ( LOCALISATION_TEXT_No_ACK_received, "NO ACK received" ),
```

'LOCALE\_EN' should be replaced with the appropriate macro for that specific culture and data file.

> **NOTE**: Be sure to place your new value just before the **LOCALISATION\_FINAL** entry, since that value is used to indicate the end of the lookup table and calculate it's total size.

## Adding a New Language/Culture##

To add a new language or culture to the system, there are four main steps that you need to follow:

**Step One: Add a new culture_t entry in localisation.h**

You need to add a new, unique entry in the culture_t enum in **localisation.h** to enable you to distinguish this culture from any other cultures existing in your firmware.  Simply add a new value in the same style as the existing entries:

```
/* Supported languages/cultures */
typedef enum
{
  CULTURE_EN,
  CULTURE_FR,
  CULTURE_COUNT
} culture_t;
```

> **NOTE**: The new culture code must be placed before the **CULTURE_COUNT** value, since this value is used to determine how many cultures are present in the system.

**Step Two: Define a macro for this culture in localisation.c**

Based on the culture code that you defined above, open the **localisation.c** file and create a new macro at the top of the file in the following format:

```
#define LOCALE_EN(key, en) [CULTURE_EN][key]=en
#define LOCALE_FR(key, fr) [CULTURE_FR][key]=fr
```

This macro will be used to associate entries in the localisation data file we will create in step three below.

**Step Three: Create a new localisation data file**

Next you need to create a locale_xx.dat file that implements all the of localisation keys in localisedTextKeys_t.

For convenience sake, you can simply copy and rename an existing file in one language, and then change all of the LOCALE_xx references to the macro name that you added in step two above before translating the text.

> **WARNING**: Unicode characters are the default on most modern text editors, but most small embedded systems still use 8-bit ANSI encoding, and you should convert any files you edit to ANSI encoding before saving them.  Free tools such as Notepad++ are capable of doing this while preserving any supported accent or special characters.

**Step Four: Reference the localisation data file in localisation.c**

Once this file has been created, you need to add it to the data file list in **localisation.c** in the following format:

```
/* Make sure the language data files are in same order as culture_t! */
const char* localised_strings[CULTURE_COUNT][LOCALISATION_FINAL+1] =
{
  #include "localisation/locale_en.dat"
  #include "localisation/locale_fr.dat"
};
```

Rebuild your code base and the new localised values should be available in your system!
