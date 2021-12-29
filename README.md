# README #

## Recommended installation
Visual Studio Code + PlatformIO
Default platform setup:
Arduino Nano - Mega 328

## Code
Code is inside src/main.cpp

## Upload code cli
```
pio run -e nanoatmega328 -t upload
``` 

## Monitor

```
pio run -e nanoatmega328 -t monitor
```

## Complete run from cli

```
pio run -e nanoatmega328 -t upload
pio run -e nanoatmega328 -t monitor
```

## Releases
Compiled binaries are placed in builds/ directory.