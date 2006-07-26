#!/bin/bash

# script to convert a binary fw package (output from gen_fw)
# into a .h file for static inclusion
# call fwbin2h.sh input_file output_file

if [ $# -ne 2 ]; then
        echo "Convert a binary firmware file into a .h file"
        echo "Usage: $0 input_file output_file"
        exit 1
fi

INF=$1
OUTF=$2

echo '/*' > $OUTF
sed 's/^/ * /' COPYRIGHT >> $OUTF
echo ' */' >> $OUTF
echo '' >> $OUTF
echo 'static u8 fw_bin[] = {' >> $OUTF

od -An -tx1 -v -w8 $INF | sed 's/\([0-9a-f][0-9a-f]\)/0x\1,/g' >> $OUTF
echo '};' >> $OUTF
echo '' >> $OUTF
echo "static const struct firmware static_fw = {`ls -l $INF | awk '{print $5}'`, fw_bin};" >> $OUTF
echo '' >> $OUTF

