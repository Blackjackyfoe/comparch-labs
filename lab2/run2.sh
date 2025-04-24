#!/bin/bash
sudo iverilog -g2012 alu_test.v && vvp a.out

for i in {1..5}; do
    echo
done

echo "============"

for i in {1..5}; do
    echo
done


sudo iverilog -g2012 register_file_test.v && vvp a.out

for i in {1..5}; do
    echo
done

echo "============"

for i in {1..5}; do
    echo
done


sudo iverilog -g2012 counter_test.v && vvp a.out