sess_ok:        ld a,(pk_db)
                or a
                jr z,so_no
                ld hl,(pk_game)
                ld a,h
                and l
                inc a
                jr z,so_no
                ld hl,pk_sid
                ld de,pk_id
                ld b,4
so_l:           ld a,(de)
                cp (hl)
                ret nz
                inc hl
                inc de
                djnz so_l
                ret
so_no:          or 1
                ret

ptr0:           ld bc,P_PTR
                out (c),a
                ret

set_bank:       ld bc,P_BANK
                out (c),a
                ret

sd_io:          push af
                ld bc,P_LBA0
                out (c),l
                ld bc,P_LBA1
                out (c),h
                xor a
                ld bc,P_LBA2
                out (c),a
                ld bc,P_LBA3
                out (c),a
                ld bc,P_COUNT
                out (c),d
                pop af
                ld bc,P_STAT
                out (c),a
sd_wait:        in a,(c)
                bit ST_BUSY,a
                jr nz,sd_wait
                and 1<<ST_ERR
                ret z
                scf
                ret

min_hl_bc:      push hl
                or a
                sbc hl,bc
                pop hl
                ret c
                ld h,b
                ld l,c
                ret

out_zeros:      xor a
                out (P_DATA),a
                dec de
                ld a,d
                or e
                jr nz,out_zeros
                ret

skip_data:      in a,(P_DATA)
                dec de
                ld a,d
                or e
                jr nz,skip_data
                ret

wait_frame:     ld bc,P_FRAME
                in a,(c)
                ld l,a
wf_l:           in a,(c)
                cp l
                jr z,wf_l
                ret

read_input:     ld d,0
                ld bc,#FDFE
                in a,(c)
                bit 1,a
                jr nz,ri_1
                set EV_SAVE,d
ri_1:           ld bc,#BFFE
                in a,(c)
                bit 1,a
                jr nz,ri_2
                set EV_LOAD,d
ri_2:           ld bc,#F7FE
                in a,(c)
                bit 4,a
                jr nz,ri_3
                set EV_PREV,d
ri_3:           ld bc,#EFFE
                in a,(c)
                bit 3,a
                jr nz,ri_4
                set EV_PREV,d
ri_4:           bit 4,a
                jr nz,ri_5
                set EV_NEXT,d
ri_5:           bit 2,a
                jr nz,ri_6
                set EV_NEXT,d
ri_6:           ld bc,#7FFE
                in a,(c)
                bit 3,a
                jr nz,ri_6n
                set EV_NAME,d
ri_6n:          bit 0,a
                jr nz,ri_6p
                ld bc,#FEFE
                in a,(c)
                bit 0,a
                jr nz,ri_6p
                set EV_BACK,d
ri_6p:          ld bc,#DFFE
                in a,(c)
                bit 0,a
                jr nz,ri_7
                set EV_POKES,d
ri_7:           ld bc,P_INPUT
                in a,(c)
                ld c,0
                bit 7,a
                jr z,ri_8
                set EV_BACK,d
ri_8:           bit 1,a
                jr z,ri_9
                set EV_PREV,c
ri_9:           bit 0,a
                jr z,ri_10
                set EV_NEXT,c
ri_10:          bit 2,a
                jr z,ri_11
                set EV_SAVE,c
ri_11:          bit 3,a
                jr z,ri_12
                set EV_LOAD,c
ri_12:          bit 4,a
                jr z,ri_12p
                set EV_NAME,c
ri_12p:         bit 5,a
                jr z,ri_13
                set EV_POKES,c
ri_13:          bit 6,a
                ret z
                set EV_BACK,c
                ret

get_event:      call wait_frame
                call read_input
                ld a,d
                or c
                ld e,a
                ld a,(in_old)
                cpl
                and e
                ld h,a
                ld a,e
                ld (in_old),a
                and (1<<EV_PREV)|(1<<EV_NEXT)
                jr z,ge_norep
                ld a,(rep_cnt)
                inc a
                ld (rep_cnt),a
                cp 20
                jr c,ge_any
                ld a,16
                ld (rep_cnt),a
                ld a,e
                and (1<<EV_PREV)|(1<<EV_NEXT)
                or h
                ld h,a
                jr ge_any
ge_norep:       xor a
                ld (rep_cnt),a
ge_any:         ld a,h
                or a
                jr z,get_event
                and d
                ld a,0
                jr nz,ge_dev
                inc a
ge_dev:         ld (byjoy),a
                ld a,h
                ret

get_key:        call wait_frame
                call scan_keys
                ld e,a
                ld a,(jold)
                cpl
                and e
                ld d,a
                ld a,e
                ld (jold),a
                ld a,K_CANCEL
                bit 7,d
                jp nz,gk_kbd
                bit 6,d
                jp nz,gk_joy
                ld a,K_ALT
                bit 5,d
                jp nz,gk_joy
                ld a,K_OK
                bit 4,d
                jp nz,gk_joy
                ld a,K_UP
                bit 3,d
                jp nz,gk_joy
                ld a,K_DOWN
                bit 2,d
                jp nz,gk_joy
                ld a,K_LEFT
                bit 1,d
                jp nz,gk_joy
                ld a,K_RIGHT
                bit 0,d
                jp nz,gk_joy
                ld hl,know
                ld de,kold
                ld b,8
gk_new:         ld a,(de)
                cpl
                ld c,a
                ld a,(hl)
                ld (de),a
                and c
                ld (hl),a
                inc hl
                inc de
                djnz gk_new
                ld hl,know
                res 0,(hl)
                ld hl,know+7
                res 1,(hl)
                ld hl,know
                ld c,0
                ld b,8
gk_find:        ld a,(hl)
                or a
                jr nz,gk_bits
                inc hl
                ld a,c
                add a,5
                ld c,a
                djnz gk_find
                xor a
                ret
gk_bits:        rrca
                jr c,gk_code
                inc c
                jr gk_bits
gk_code:        ld b,0
                ld hl,keymap
                add hl,bc
                ld a,(hl)
                cp K_OK
                jr z,gk_kbd
                ld e,a
                ld hl,sym_map
                ld a,(kold+7)
                bit 1,a
                ld a,e
                jr nz,gk_map
                ld hl,caps_map
                ld a,(kold)
                bit 0,a
                ld a,e
                jr z,gk_kbd
                cp 'a'
                jr c,gk_map
                sub 32
                jr gk_kbd
gk_map:         ld b,a
gk_ml:          ld a,(hl)
                or a
                ret z
                inc hl
                cp b
                ld a,(hl)
                inc hl
                jr nz,gk_ml
gk_kbd:         ld e,a
                xor a
                ld (byjoy),a
                ld a,e
                ret
gk_joy:         ld e,a
                ld a,1
                ld (byjoy),a
                ld a,e
                ret

key_sync:       call scan_keys
                ld (jold),a
                ld hl,know
                ld de,kold
                ld bc,8
                ldir
                ret

scan_keys:      ld hl,know
                ld bc,#FEFE
sk_l:           in a,(c)
                cpl
                and #1F
                ld (hl),a
                inc hl
                rlc b
                jr c,sk_l
                ld bc,P_INPUT
                in a,(c)
                ret

wait_release:   call wait_frame
                call read_input
                ld a,d
                or c
                jr nz,wait_release
                ld bc,#00FE
                in a,(c)
                and #1F
                cp #1F
                jr nz,wait_release
                ld bc,P_INPUT
                in a,(c)
                or a
                jr nz,wait_release
                ret

cls:            ld hl,SCREEN
                ld de,SCREEN+1
                ld bc,6911
                ld (hl),0
                ldir
                ret

clear_line:     ld hl,line
                ld b,32
cl_l:           ld (hl),' '
                inc hl
                djnz cl_l
                ret

print_row:      push bc
                ld e,0
pr_l:           ld a,(hl)
                push hl
                push de
                ld d,b
                call put_char
                pop de
                pop hl
                inc hl
                inc e
                ld a,e
                cp 32
                jr nz,pr_l
                pop bc
                ld l,b
                ld h,0
                add hl,hl
                add hl,hl
                add hl,hl
                add hl,hl
                add hl,hl
                ld de,ATTRS
                add hl,de
                ld b,32
pr_a:           ld (hl),c
                inc hl
                djnz pr_a
                ret

put_char:       push bc
                sub 32
                jr nc,pc_1
                xor a
pc_1:           cp 96
                jr c,pc_2
                xor a
pc_2:           ld l,a
                ld h,0
                add hl,hl
                add hl,hl
                add hl,hl
                ld bc,font
                add hl,bc
                push hl
                ld a,d
                and #18
                or SCREEN >> 8
                ld h,a
                ld a,d
                and 7
                rrca
                rrca
                rrca
                or e
                ld l,a
                pop de
                ld b,8
pc_l:           ld a,(de)
                ld (hl),a
                inc de
                inc h
                djnz pc_l
                pop bc
                ret

dec2:           ld b,'0'-1
d2_l:           inc b
                sub 10
                jr nc,d2_l
                add a,10+'0'
                ld (hl),b
                inc hl
                ld (hl),a
                inc hl
                ret

bcd2:           push af
                rrca
                rrca
                rrca
                rrca
                and 15
                add a,'0'
                ld (hl),a
                inc hl
                pop af
                and 15
                add a,'0'
                ld (hl),a
                inc hl
                ret

keymap:         db 0, "zxcv"
                db "asdfg"
                db "qwert"
                db "12345"
                db "09876"
                db "poiuy"
                db K_OK, "lkjh"
                db ' ', 0, "mnb"
caps_map:       db '5', K_LEFT, '8', K_RIGHT, '7', K_UP, '6', K_DOWN, '0', K_DEL, ' ', K_CANCEL, 0
sym_map:        db "j-n,m.7'", 0

letters:        db " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-."
