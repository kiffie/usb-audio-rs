#!/bin/bash

BIN=usb-audio

#BUILD_FLAGS="--release --features adau1961,ssd1306"
BUILD_FLAGS="--release --features ssd1306"

DUMP_LST=""
RUN_DFU=""
PRINT_USAGE=""
while getopts "lrh" opt; do
    case $opt in
        (l)
            DUMP_LST=1
            ;;
        (r)
            RUN_DFU=1
            ;;
        (h)
            PRINT_USAGE=1
            ;;
    esac
done

if [ "$PRINT_USAGE" ]; then
    echo "Usage: $0 [-l] [-r] [-h]"
    echo "  -l: dump listing"   
    echo "  -r: run DFU and usb-logread"
    echo "  -h: print usage"
    exit
fi

git submodule update --init
cargo build $BUILD_FLAGS || exit

if [ "$DUMP_LST" ]; then
	rust-objdump -sDt --demangle target/mipsel-unknown-none/release/$BIN > $BIN.lst || exit
fi

cargo objcopy $BUILD_FLAGS -- -O ihex $BIN.hex
cargo objcopy $BUILD_FLAGS -- -R .bootloader -R .configsfrs -O ihex $BIN.app.hex
cargo objcopy $BUILD_FLAGS -- -R .bootloader -R .configsfrs -O binary $BIN.app.bin

if [ $(basename $0) = "run.sh" -o "$RUN_DFU" ]; then
    dfu-util -R -D $BIN.app.bin
    sleep 1
    usb-logread
fi
