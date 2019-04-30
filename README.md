Navio 2
=====

Collection of drivers and examples for Navio 2 - autopilot shield for Raspberry Pi. 
Has been forked due to bugs in original code. 

This code still has a lot of bugs in it, which will be (hopefully) corrected in the near future.

### Problem with pigpio.h

If you have got error: `fatal error: pigpio.h: No such file or directory`, install `pigpio` by commands:

    sudo apt-get update
    sudo apt-get install pigpio
