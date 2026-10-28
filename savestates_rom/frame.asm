                org #0000
                ds #100, 0

                org #0100
entry:          di
                ld sp,STACK
                jp main

                org #0180
to_page:        ld sp,STACK
                ld bc,P_ROMPG
                out (c),a
                jp main

                ds #0400-$, 0
font:           incbin "font.bin"

                org #0700
