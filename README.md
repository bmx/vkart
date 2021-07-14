# V.Kart programmer 

This tool is the firmware to flash to the msp432. It's used to program or read v.kart/V.Smile cartriges.

To build it:

create `Release/config.mk` based on `Release/config.mk.sample` to reflect ccs local path and compiler path.

go to `Release` and type `make`

it will produce a `vkart.out` elf file

# to flash the firmware

Use uniflash gui to create a command line package
Once created and unzipped, edit `uniflash/dslite.sh` to get rid of the hardcoded path and use `$1` instead.

then, use `uniflash/dslite.sh <pathto>/vkart.out`

# to access the tool

Connect to the main serial port (`/dev/ttyACM0`)
```
$ screen /dev/ttyACM0 115200
```

A menu should appear. Use `?` to redisplay the menu eventualy
