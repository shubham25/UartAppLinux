# UartAppLinux
Uart Application developed for RS485 Communication on BeagleBone Black, using termios.
- Modular based interface
- pthread and termios implementation
- stty

## File Details
- **compileUART.sh**  : 

    Simple g++ compiler commands to build app.
    
- **MakeFile**  :

    This can also be used to build app. ( Preferred )
    
- **multiUART.cpp**  : 

  *BBB provides 4 Uart ( 1,2,4,5) . Uart0 is used for boot messages. This app runs them all in parallel using thread having read/write fxns.*
    
- **multiUARTApp.cpp**  : 

    2 thread based app for testing the application, as required in the Project. This is similar to above file, and more specific to the Project specs.
    
- **uartconfig.txt**  :
    
    Uart configuration used for termios structure
    
- **uartRx.cpp** :  

    A simple Uart receive module prototype for testing RX case
    
- **uartTx.cpp** : 
    
    A simple Uart transmit module prototype for testing TX case
    
- **uartTxLoop.cpp** - 
    
    Test a continuous transmission case
 
 - **uartPinsConfig.sh** - 
    
    Pin configuration for all UARTs , this is specfic to BeagleBone black and may vary.

## Usage / Setup :
1. **Compile** :
 
You can either use compileuart or makefile , since it is a small project, it doesn't matter much. The variable "Target" has to be changed in MAKEFILE for different App as required.

*Run*
```
chmod a+x compileUART.sh
./compileUART.sh
```

*OR*
```
make clean
make
```
2. **Pin Configurtion** :

For BeagleBone Black, if the cape-overlay is loaded, config-pin can be directly used to set pins for UART. Else corresponding BB-UART1-00A0.dtbo like devicetree overlays has to be added in uEnv.txt file. 

*Run*
```
chmod a+x uartPinsConfig.sh
./uartPinsConfig.sh
```
- *Now you can execute the application*


##TODO
* multiUARTApp.cpp 
    - [ ] Implement mutex-based lock in threads.


## References
The following links provide a very good read about Serial programming and termios structure details.

* [HOWTO-Serial-Programming](http://tldp.org/HOWTO/Serial-Programming-HOWTO/x115.html) - Also include sample code. Good explanation of canonical and non-canonical modes.
* [Serial_Programming-wiki](https://en.wikibooks.org/wiki/Serial_Programming/termios)
* [termios](https://blog.nelhage.com/2009/12/a-brief-introduction-to-termios-termios3-and-stty/) - good explanation of termios flags
* [stty](https://linux.101hacks.com/unix/stty/) and [man page](https://linux.die.net/man/1/stty)
