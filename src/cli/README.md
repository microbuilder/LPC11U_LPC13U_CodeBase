# Command Line Interface (CLI) #

The code base includes a basic, easy to extend command-line interface (CLI) that provides a convenient way for end-users to interact with or debug your project or hardware.

The CLI uses printf for the output, so it can be configured to work with either **UART** or **USB CDC** depending on the CFG_PRINTF settings in your board config file.

Incoming characters are handled via the appropriate interrupt (UART or USB CDC), and text is held in a buffer until **cliPoll()** is called, which will attempt to process any valid commands.

## Using the CLI ##

Depending on if you are using UART or USB CDC for the CLI, you need to open some sort of terminal software (for example Tera Term on Windows), set it to the correct COM port, and set the baud rate to **115200** (or whatever baud rate you have set in your board config file if you are using UART).

You can enter **'?'** followed by the carriage-return character to display a list of all commands available on the system, or enter any command followed by '?' to see a basic description of the command and it's parameters.

## Extending the CLI ##

Adding new commands to the CLI is relatively easy.  There are two simple steps to follow:

**Step One: Create a new command in 'cli/commands'**

The first thing you need to do is implement the command in a seperate file that should be placed in the '/cli/commands' folder, following the same model at the existing commands:

```
/**************************************************************************/
/*!
    Your command handler
*/
/**************************************************************************/
void cmd_yourCommand(uint8_t argc, char **argv)
{
  /* argc = the number of supplied arguments  */
  /* argv[x] = the specific argument contents */

  printf("Your command!%s", CFG_PRINTF_NEWLINE);
}
```

**Step Two: Add the command to the command table (cli/cli_tbl.h)**

Once your command has been created, you simply need to insert it in the command lookup table with a few parameters that let the system know how to handle the command and perform some basic error-checking.

At the top of cli_tbl.h add the function prototype for your command, similar to the following:

```
void cmd_yourCommand(uint8_t argc, char **argv);
```

And then add a new command entry in the cli_tbl array:

```
{ "cmd", 1,  2,  0, cmd_yourCommand , "Desc of your command" , "'cmd param1 [<param2>]'" },
```

The parameters in the lookup table are the following (in order of appearance):

- **Command name**: The text that will be associated with your command.  This should be a single world (no spaces or special characters) since the command parameters are parsed based on the 'space' character.
- **Minimum arguments**: The minimum number of arguments that are required for this command (not including the command name itself).  You must have at least this many arguments present or the CLI will reject the command input.
- **Maximum arguments**: The maximum number of arguments if you wish to support some optional arguments as well.  Anything with more arguments than the max value will also be rejected by the CLI.
- **Hidden**: If this is set to '1' then the command will not appear in the command list (entering the '?' character at the command prompt).
- **Function name**: the name of the function where you command is implemented (ex.: 'cmd_yourCommand').  This is where the CLI will redirect command processing to if it passed basic validation checks.
- **Description**: This is a short description of your command that will appear in the command list, and in the help menu (when you enter the command name followed by the '?' character).
- **Syntax**: The command syntax, including the cmd name, mandatory parameters, and optional parameters (the latter should be placed in square brackets to indicate they are optional).