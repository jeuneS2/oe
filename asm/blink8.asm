// blink LEDs at 0.5 Hz
// assumes acc=8, reg=8, 100MHz, Leds @ address 0

// [0] = LED status
// [3:2:1] = counter

        // define initial status of LEDs
        ldi 0
        sto 0

        // set address and mask registers
        ldi 0
        xad
        ldi -1
        xsm        

        // 100 000 000 / 17 == 0x 59 c1 f1
        ldi 5
        ldl 9
        sto 3
        ldi 12
        ldl 1
        sto 2
        ldi 15
        ldl 1
        sto 1

        // the main loop
        ldi -1 // [3:2:1] -= 1
        add 1
        sto 1
        ldi -1
        adc
        add 2        
        sto 2
        ldi -1
        adc
        add 3        
        sto 3
        
        ior 2 // ([3] | [2] | [1]) == 0
        ior 1
        cmp
        ldi 0
        ldl 15
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