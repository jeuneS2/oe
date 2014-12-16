// blink LEDs at 0.5 Hz
// assumes acc=32, reg=32, 100MHz, Leds @ address 0

// [0] = LED status
// [1] = counter

        // define initial status of LEDs
        ldi 0
        sto 0

        // set address and mask registers
        ldi 0
        xad
        ldi -1
        xsm        

        // 100 000 000 / 6 == 0x fe 50 2b
        ldi 0
        ldl 15
        ldl 14
        ldl 5
        ldl 0
        ldl 2
        ldl 11
        sto 1

        // the main loop
        ldi -1 // [1] -= 1
        add 1
        sto 1
        cmp
        ldi 0
        ldl 14
        jnz
        nop

        ldi 1
        add 0
        sto 0
        xsd
        xsa
        ldi 6
        jmp
        nop
