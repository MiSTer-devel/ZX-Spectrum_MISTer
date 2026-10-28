                include "defs.asm"

M_EMPTY         equ 1
M_NOSAVE        equ 2
M_NOLOAD        equ 3
M_1024          equ 4
M_NEED128       equ 5
M_NEEDP3        equ 6
M_NEEDSCORP     equ 7
M_NEED48        equ 8
M_RO            equ 9
M_NONAME        equ 10
M_NOPOKES       equ 11

                include "frame.asm"

main:           call read_sb
                xor a
                ld (msg),a
                ld (confirm),a
                ld (byjoy),a
                ld bc,P_BORDER
                out (c),a
                call cls
                call read_input
                ld a,d
                or c
                ld (in_old),a
                ld a,c
                or a
                jr z,m_kbd
                ld a,1
                ld (byjoy),a
m_kbd:          ld bc,P_LOADS
                in a,(c)
                ld hl,pk_loads
                cp (hl)
                jr z,m_same
                ld (hl),a
                ld hl,#FFFF
                ld (pk_game),hl
m_same:         ld hl,gk_crc
                ld bc,P_CRC
                ld d,4
m_crc:          in a,(c)
                ld (hl),a
                inc hl
                inc b
                dec d
                jr nz,m_crc
                ld bc,P_GAME
                in a,(c)
                ld (hl),a
                call disk_check
                ld a,(slot)
                ld hl,nslots
                cp (hl)
                jr c,browse
                xor a
                ld (slot),a

browse:         call show_slot
loop:           call get_event
                ld b,a
                bit EV_BACK,b
                jr nz,on_back
                bit EV_PREV,b
                jr nz,on_prev
                bit EV_NEXT,b
                jr nz,on_next
                bit EV_SAVE,b
                jr nz,on_save
                bit EV_LOAD,b
                jr nz,on_load
                bit EV_NAME,b
                jp nz,on_name
                bit EV_POKES,b
                jp nz,on_pokes
                jp loop

on_back:        ld a,(confirm)
                or a
                jp z,leave
                xor a
                ld (confirm),a
                call draw_help
                jp loop

on_prev:        ld a,(slot)
                or a
                jr nz,ep1
                ld a,(nslots)
ep1:            dec a
                ld (slot),a
                jr new_slot

on_next:        ld a,(slot)
                inc a
                ld hl,nslots
                cp (hl)
                jr c,en1
                xor a
en1:            ld (slot),a
new_slot:       xor a
                ld (confirm),a
                ld (msg),a
                jp browse

on_save:        xor a
                ld (msg),a
                ld a,(valid)
                or a
                jr z,es_go
                ld a,(confirm)
                cp 1
                jr z,es_go
                ld a,1
                ld (confirm),a
                call draw_help
                jp loop
es_go:          xor a
                ld (confirm),a
                call do_save
                jp nc,leave
                jp browse

on_load:        xor a
                ld (msg),a
                ld a,(valid)
                or a
                jr nz,el1
                ld a,M_EMPTY
                ld (msg),a
                call draw_help
                jp loop
el1:            ld a,(confirm)
                cp 2
                jr z,el2
                ld a,2
                ld (confirm),a
                call draw_help
                jp loop
el2:            xor a
                ld (confirm),a
                call do_load
                jp nc,leave
                call draw_help
                jp loop

on_name:        xor a
                ld (confirm),a
                ld (msg),a
                ld a,(valid)
                or a
                ld a,M_EMPTY
                jr z,nm_msg
                ld a,(ro)
                or a
                ld a,M_RO
                jr nz,nm_msg
                call edit_name
                jr c,nm_done
                call write_name
                jp c,browse
nm_done:        call read_input
                ld a,d
                or c
                ld (in_old),a
                call draw_name
                call draw_help
                jp loop
nm_msg:         ld (msg),a
                call draw_help
                jp loop

on_pokes:       xor a
                ld (confirm),a
                ld (msg),a
                ld a,(pk_db)
                or a
                ld a,M_NOPOKES
                jr z,nm_msg
                ld a,1
                jp to_page

leave:          call wait_release
                ld bc,P_LEAVE
                out (c),a
leave_wait:     jr leave_wait

disk_check:     xor a
                ld (pk_db),a
                ld bc,P_STAT
                in a,(c)
                ld e,a
                xor a
                bit ST_RO,e
                jr z,dc_rw
                inc a
dc_rw:          ld (ro),a
                bit ST_DISK,e
                jp z,no_disk
                call ptr0
                ld hl,0
                ld d,0
                ld a,CMD_READ
                call sd_io
                jp c,bad_disk
                call ptr0
                ld hl,hdr
                ld b,16
                ld c,P_DATA
                inir
                ld hl,hdr
                ld de,disk_hdr
                ld b,16
dc_cmp:         ld a,(de)
                cp (hl)
                jr nz,dc_diff
                inc hl
                inc de
                djnz dc_cmp
                jr dc_slots
dc_diff:
                ld hl,hdr
                ld b,16
                xor a
dc_or1:         or (hl)
                inc hl
                djnz dc_or1
                ld e,a
                ld hl,496
dc_or2:         in a,(P_DATA)
                or e
                ld e,a
                dec hl
                ld a,h
                or l
                jr nz,dc_or2
                ld a,e
                or a
                jp nz,bad_disk
                ld a,(ro)
                or a
                jp nz,bad_disk
                call ptr0
                ld hl,disk_hdr
                ld b,16
                ld c,P_DATA
                otir
                ld de,496
                call out_zeros
                ld hl,0
                ld d,0
                ld a,CMD_WRITE
                call sd_io
                jp c,bad_disk
dc_slots:
                ld bc,P_LBA2
                in a,(c)
                ld e,a
                ld bc,P_LBA3
                in a,(c)
                or e
                ld hl,#FFFF
                jr nz,dc_big
                ld bc,P_LBA1
                in h,(c)
                ld bc,P_LBA0
                in l,(c)
dc_big:         ld de,FIRST_LBA+SLOT_SECS
                ld c,0
dc_n:           push hl
                or a
                sbc hl,de
                pop hl
                jr c,dc_end
                inc c
                ld a,c
                cp 64
                jr z,dc_end
                push hl
                ld hl,SLOT_SECS
                add hl,de
                ex de,hl
                pop hl
                jr dc_n
dc_end:         ld a,c
                ld (nslots),a
                or a
                jp z,bad_disk

                ld hl,POKES_LBA
                ld d,0
                ld a,CMD_READ
                call sd_io
                ret c
                call ptr0
                ld hl,hdr
                ld b,32
                ld c,P_DATA
                inir
                ld hl,hdr
                ld de,t_zxpokes
                ld b,8
pd_cmp:         ld a,(de)
                cp (hl)
                ret nz
                inc hl
                inc de
                djnz pd_cmp
                ld hl,(hdr+8)
                ld a,h
                or l
                ret z
                ld (pk_count),hl
                ld hl,(hdr+12)
                ld (pk_ixlba),hl
                ld hl,(hdr+16)
                ld (pk_dtlba),hl
                ld hl,(hdr+20)
                ld (pk_dtsecs),hl
                ld hl,(hdr+24)
                ld (pk_id),hl
                ld hl,(hdr+26)
                ld (pk_id+2),hl
                ld a,1
                ld (pk_db),a
                ret

no_disk:        ld hl,t_nodisk
                jr disk_msg
bad_disk:       ld hl,t_baddisk
disk_msg:       push hl
                call cls
                ld b,0
                ld hl,t_title
                ld c,A_BAR
                call print_row
                pop hl
                ld b,22
                ld c,A_NAME
                call print_row
                ld b,23
                ld hl,t_anykey
                ld c,A_BAR
                call print_row
                pop hl
                call wait_release
dm_wait:        call wait_frame
                call read_input
                ld a,d
                or c
                jr nz,dm_go
                ld bc,#00FE
                in a,(c)
                and #1F
                cp #1F
                jr z,dm_wait
dm_go:          jp leave

show_slot:      call slot_lba
                ld (slotbase),hl
                call ptr0
                ld d,14
                ld a,CMD_READ
                call sd_io
                jr nc,ss_read
                xor a
                ld (valid),a
                jr ss_draw
ss_read:        call ptr0
                ld hl,hdr
                ld b,64
                ld c,P_DATA
                inir
                ld de,448
                call skip_data
                ld hl,hdr
                ld de,t_zxss
                ld b,4
ss_cmp:         ld a,(de)
                cp (hl)
                jr nz,ss_bad
                inc hl
                inc de
                djnz ss_cmp
                ld a,(hdr+4)
                cp 1
                jr nz,ss_bad
                ld a,(hdr+5)
                cp 1
                jr nz,ss_bad
                ld a,1
                jr ss_val
ss_bad:         xor a
ss_val:         ld (valid),a
                ld a,(hdr+6)
                ld (fmt),a
                ld a,(hdr+7)
                ld (arch),a
                ld hl,hdr+8
                ld de,slen
                ld bc,3
                ldir
                ld hl,hdr+16
                ld de,srtc
                ld bc,8
                ldir
                ld a,(hdr+24)
                and 7
                ld (sborder),a
                ld hl,hdr+32
                ld de,sname
                ld bc,32
                ldir
ss_draw:        ld a,(valid)
                or a
                jr nz,ss_pic
                call cls
                ld hl,sname
                ld de,sname+1
                ld bc,31
                ld (hl),0
                ldir
                xor a
                ld (sborder),a
                jr ss_rows
ss_pic:         ld hl,SCREEN
                ld c,P_DATA
                ld e,27
ss_copy:        ld b,0
                inir
                dec e
                jr nz,ss_copy
ss_rows:        ld a,(sborder)
                ld bc,P_BORDER
                out (c),a
                call draw_title
                call draw_name
                jp draw_help

draw_title:     call clear_line
                ld hl,line
                ld (hl),'S'
                inc hl
                ld (hl),'L'
                inc hl
                ld (hl),'O'
                inc hl
                ld (hl),'T'
                ld hl,line+5
                ld a,(slot)
                inc a
                call dec2
                ld a,(valid)
                or a
                jr z,dt_print
                ld hl,srtc
                ld b,6
                xor a
dt_any:         or (hl)
                inc hl
                djnz dt_any
                jr z,dt_model
                ld hl,line+9
                ld a,(srtc+3)
                call bcd2
                ld (hl),'/'
                inc hl
                ld a,(srtc+4)
                call bcd2
                ld (hl),'/'
                inc hl
                ld (hl),'2'
                inc hl
                ld (hl),'0'
                inc hl
                ld a,(srtc+5)
                call bcd2
                inc hl
                ld a,(srtc+2)
                call bcd2
                ld (hl),':'
                inc hl
                ld a,(srtc+1)
                call bcd2
dt_model:       ld a,(arch)
                call model_text
                ld de,line+27
                ld bc,5
                ldir
dt_print:       ld b,0
                ld hl,line
                call title_attr
                jp print_row

title_attr:     ld c,A_BAR
                ld a,(valid)
                or a
                ret z
                ld a,(gk_known)
                or a
                ret z
                ld a,(hdr+30)
                dec a
                ret nz
                push hl
                push de
                push bc
                ld hl,hdr+26
                ld de,gk_crc
                ld b,4
ta_l:           ld a,(de)
                cp (hl)
                jr nz,ta_no
                inc hl
                inc de
                djnz ta_l
                pop bc
                ld c,A_GAME
                jr ta_end
ta_no:          pop bc
ta_end:         pop de
                pop hl
                ret

model_text:     ld c,a
                rrca
                rrca
                and 7
                ld hl,t_m48
                cp 3
                ret z
                ld hl,t_mp3
                cp 4
                ret z
                ld hl,t_mscorp
                cp 5
                ret z
                ld hl,t_mnone
                or a
                ret nz
                ld hl,t_m128
                ld a,c
                and 3
                cp 2
                ret nz
                ld hl,t_mpent
                ret

draw_name:      call clear_line
                ld a,(valid)
                or a
                jr nz,dn_name
                ld hl,t_empty
                jr dn_print
dn_name:        ld hl,sname
                ld de,line+1
                ld b,31
dn_copy:        ld a,(hl)
                cp 32
                jr c,dn_sp
                cp 127
                jr c,dn_ok
dn_sp:          ld a,' '
dn_ok:          ld (de),a
                inc hl
                inc de
                djnz dn_copy
                ld hl,line
dn_print:       ld b,22
                ld c,A_NAME
                jp print_row

draw_help:      ld a,(msg)
                or a
                jr z,dh_ask
                dec a
                add a,a
                ld l,a
                ld h,0
                ld de,msg_table
                add hl,de
                ld a,(hl)
                inc hl
                ld h,(hl)
                ld l,a
                ld c,A_BAR
                jr dh_print
dh_ask:         ld a,(confirm)
                or a
                jr z,dh_keys
                ld c,A_ASK
                ld hl,t_ovr_key
                cp 1
                jr z,dh_dev
                ld hl,t_ld_key
dh_dev:         ld a,(byjoy)
                or a
                jr z,dh_print
                ld de,32
                add hl,de
                jr dh_print
dh_keys:        ld c,A_BAR
                ld hl,t_keys
                ld a,(byjoy)
                or a
                jr z,dh_pokes
                ld hl,t_joy
dh_pokes:       ld a,(pk_db)
                or a
                jr z,dh_print
                ld de,64
                add hl,de
dh_print:       ld b,23
                jp print_row

status_line:    ld b,23
                ld c,A_BAR
                jp print_row

edit_name:      ld hl,sname
                ld de,ename
                ld bc,32
                ldir
                call name_size
                cp NAME_LEN
                jr c,ed_cur
                ld a,NAME_LEN-1
ed_cur:         ld (ecur),a
                call key_sync
ed_draw:        call draw_edit
ed_key:         call get_key
                or a
                jr z,ed_key
                cp K_CANCEL
                jp z,ed_cancel
                cp K_OK
                jp z,ed_ok
                cp K_ALT
                jr z,ed_key
                ld e,a
                ld a,(ecur)
                ld c,a
                ld b,0
                ld hl,ename
                add hl,bc
                ld a,e
                cp K_LEFT
                jr z,ed_left
                cp K_RIGHT
                jr z,ed_right
                cp K_UP
                jr z,ed_up
                cp K_DOWN
                jr z,ed_down
                cp K_DEL
                jr z,ed_del
                ld (hl),a
ed_right:       ld a,c
                cp NAME_LEN-1
                jr nc,ed_draw
                inc a
ed_setc:        ld (ecur),a
                jr ed_draw
ed_left:        ld a,c
                or a
                jr z,ed_draw
                dec a
                jr ed_setc
ed_del:         ld a,c
                or a
                jr z,ed_draw
                dec hl
                ld (hl),0
                dec a
                jr ed_setc
ed_up:          call letter_index
                inc a
                cp 39
                jr c,ed_put
                xor a
                jr ed_put
ed_down:        call letter_index
                or a
                jr nz,ed_dn
                ld a,39
ed_dn:          dec a
ed_put:         ld e,a
                ld d,0
                push hl
                ld hl,letters
                add hl,de
                ld a,(hl)
                pop hl
                ld (hl),a
                jr ed_draw
ed_ok:          call name_size
                ld hl,ename
                ld de,sname
                ld b,32
ed_cp:          or a
                jr z,ed_end
                dec a
                ld c,a
                ld a,(hl)
                cp ' '
                jr nc,ed_ch
                ld a,' '
ed_ch:          ld (de),a
                ld a,c
                jr ed_next
ed_end:         ld (de),a
ed_next:        inc hl
                inc de
                djnz ed_cp
                or a
                ret
ed_cancel:      scf
                ret

name_size:      ld hl,ename
                ld b,NAME_LEN
                ld c,0
                ld e,0
nl_l:           ld a,(hl)
                inc hl
                inc e
                cp 33
                jr c,nl_sp
                ld c,e
nl_sp:          djnz nl_l
                ld a,c
                ret

letter_index:   ld a,(hl)
                cp 'a'
                jr c,li_1
                cp 'z'+1
                jr nc,li_1
                sub 32
li_1:           push hl
                ld hl,letters
                ld bc,39
                cpir
                pop hl
                jr nz,li_none
                ld a,38
                sub c
                ret
li_none:        xor a
                ret

draw_edit:      ld hl,ename
                ld de,line+1
                ld b,NAME_LEN
dr_copy:        ld a,(hl)
                cp 32
                jr c,dr_sp
                cp 127
                jr c,dr_ok
dr_sp:          ld a,' '
dr_ok:          ld (de),a
                inc hl
                inc de
                djnz dr_copy
                ld a,' '
                ld (line),a
                ld (line+31),a
                ld hl,line
                ld b,22
                ld c,A_EDIT
                call print_row
                ld a,(ecur)
                inc a
                ld l,a
                ld h,0
                ld de,ATTRS+22*32
                add hl,de
                ld (hl),A_CUR
                ld hl,t_ed_key
                ld a,(byjoy)
                or a
                jr z,dr_help
                ld hl,t_ed_joy
dr_help:        ld b,23
                ld c,A_BAR
                jp print_row

write_name:     ld hl,(slotbase)
                ld d,0
                ld a,CMD_READ
                call sd_io
                jr c,wn_fail
                call ptr0
                ld de,32
                call skip_data
                ld hl,sname
                ld b,32
                ld c,P_DATA
                otir
                ld hl,(slotbase)
                ld d,0
                ld a,CMD_WRITE
                call sd_io
                ret nc
wn_fail:        ld a,M_NONAME
                ld (msg),a
                scf
                ret

do_save:        call save_model
                ret c
                ld a,(ro)
                or a
                jr z,ds_rw
                ld a,M_RO
                ld (msg),a
                scf
                ret
ds_rw:          ld hl,t_saving
                call status_line
                xor a
                call build_header
                ld hl,(slotbase)
                ld d,0
                ld a,CMD_WRITE
                call sd_io
                jp c,save_fail
                call ptr0
                ld a,(scrbank)
                call set_bank
                ld hl,#C000
                ld c,P_DATA
                ld e,27
ds_scr:         ld b,0
                otir
                dec e
                jr nz,ds_scr
                ld de,768
                call out_zeros
                ld hl,(slotbase)
                inc hl
                ld d,14
                ld a,CMD_WRITE
                call sd_io
                jp c,save_fail
                ld hl,(slotbase)
                ld de,16
                add hl,de
                ld (lba),hl
                call ptr0
                ld hl,0
                ld (bufcnt),hl
                call build_z80
                ld hl,z80h
                ld de,87
                call put_bytes
                jp c,save_fail
                xor a
                ld (blk),a
ds_blk:         ld a,(blk)
                ld hl,nblk
                cp (hl)
                jr z,ds_last
                call block_page
                push bc
                ld a,#FF
                call put_byte
                ld a,#FF
                call put_byte
                pop bc
                push bc
                ld a,b
                call put_byte
                pop bc
                jp c,save_fail
                ld a,c
                call set_bank
                ld hl,#C000
                ld de,16384
                call put_bytes
                jp c,save_fail
                ld hl,blk
                inc (hl)
                jr ds_blk
ds_last:        call flush_last
                jp c,save_fail
                ld a,1
                call build_header
                ld hl,(slotbase)
                ld d,0
                ld a,CMD_WRITE
                call sd_io
                jp c,save_fail
                or a
                ret
save_fail:      ld a,M_NOSAVE
                ld (msg),a
                scf
                ret

save_model:     ld a,(sb+30)
                ld c,a
                rrca
                rrca
                and 7
                cp 3
                jr z,sm_48
                or a
                jr z,sm_128
                cp 4
                jr z,sm_p3
                cp 5
                jr z,sm_scorp
                ld a,M_1024
                ld (msg),a
                scf
                ret
sm_48:          xor a
                ld (hw),a
                ld a,3
                ld hl,49248
                ld e,0
                jr sm_set
sm_128:         ld a,c
                and 3
                cp 2
                ld a,4
                jr nz,sm_hw
                ld a,9
                jr sm_hw
sm_p3:          ld a,7
sm_hw:          ld (hw),a
                ld a,8
                ld hl,131183 & #FFFF
                ld e,131183 >> 16
                jr sm_set
sm_scorp:       ld a,10
                ld (hw),a
                ld a,16
                ld hl,262279 & #FFFF
                ld e,262279 >> 16
sm_set:         ld (nblk),a
                ld (flen),hl
                ld a,e
                ld (flen+2),a
                ld a,5
                ld (scrbank),a
                ld a,(hw)
                or a
                ret z
                ld a,(sb+28)
                bit 3,a
                ret z
                ld a,7
                ld (scrbank),a
                or a
                ret

block_page:     ld c,a
                ld a,(hw)
                or a
                jr nz,bp_128
                ld a,c
                ld hl,pages48
                ld e,a
                ld d,0
                add hl,de
                ld b,(hl)
                ld hl,banks48
                add hl,de
                ld c,(hl)
                ret
bp_128:         ld a,c
                add a,3
                ld b,a
                ret

pages48:        db 4, 5, 8
banks48:        db 2, 0, 5

build_header:   ld e,a
                call ptr0
                ld hl,t_zxss
                ld b,4
                ld c,P_DATA
                otir
                ld a,1
                out (P_DATA),a
                ld a,e
                out (P_DATA),a
                xor a
                out (P_DATA),a
                ld a,(sb+30)
                out (P_DATA),a
                ld a,e
                or a
                jr z,bh_nolen
                ld hl,flen
                ld b,3
                otir
                jr bh_len
bh_nolen:       xor a
                out (P_DATA),a
                out (P_DATA),a
                out (P_DATA),a
bh_len:         xor a
                out (P_DATA),a
                ld hl,sb+57
                ld b,4
                otir
                ld hl,sb+49
                ld b,8
                otir
                ld a,(sb+27)
                and 7
                out (P_DATA),a
                ld a,e
                out (P_DATA),a
                ld hl,gk_crc
                ld b,5
                otir
                xor a
                out (P_DATA),a
                ld hl,sname
                ld b,32
                otir
                call sess_ok
                ld de,448
                jp nz,out_zeros
                ld a,'P'
                out (P_DATA),a
                ld a,'K'
                out (P_DATA),a
                ld hl,pk_sess
                ld c,P_DATA
                ld b,0
                otir
                ld b,SESS_LEN-256
                otir
                ld de,448-2-SESS_LEN
                jp out_zeros

build_z80:      ld hl,z80h
                ld de,z80h+1
                ld bc,95
                ld (hl),0
                ldir
                ld hl,z80map
bz_map:         ld a,(hl)
                cp #FF
                jr z,bz_fix
                ld e,a
                ld d,0
                inc hl
                ld a,(hl)
                inc hl
                push hl
                ld l,a
                ld h,0
                ld bc,sb
                add hl,bc
                ld a,(hl)
                ld hl,z80h
                add hl,de
                ld (hl),a
                pop hl
                jr bz_map
bz_fix:         ld hl,sb+32
                ld de,z80h+39
                ld bc,16
                ldir
                ld a,(sb+25)
                and #7F
                ld (z80h+11),a
                ld a,(sb+27)
                and 7
                add a,a
                ld b,a
                ld a,(sb+25)
                rlca
                and 1
                or b
                ld (z80h+12),a
                ld a,(sb+26)
                ld b,a
                rrca
                rrca
                and 1
                ld (z80h+27),a
                ld a,b
                rrca
                rrca
                rrca
                and 1
                ld (z80h+28),a
                ld a,(sb+48)
                and 1
                add a,a
                add a,a
                ld c,a
                ld a,b
                and 3
                or c
                or #40
                ld (z80h+29),a
                ld a,55
                ld (z80h+30),a
                ld a,(hw)
                ld (z80h+34),a
                ld a,4
                ld (z80h+37),a
                ld a,#FF
                ld (z80h+61),a
                ld (z80h+62),a
                ret

z80map:         db 0,0, 1,1, 2,2, 3,3, 4,6, 5,7, 8,20, 9,21, 10,24, 13,4, 14,5
                db 15,10, 16,11, 17,12, 18,13, 19,14, 20,15, 21,8, 22,9
                db 23,18, 24,19, 25,16, 26,17, 32,22, 33,23, 35,28, 38,31, 86,29
                db #FF

do_load:        ld a,(fmt)
                or a
                jr z,dl_z80
                ld a,M_NOLOAD
                ld (msg),a
                scf
                ret
dl_z80:         ld hl,t_loading
                call status_line
                ld hl,(slotbase)
                ld de,16
                add hl,de
                ld (lba),hl
                ld hl,(slen)
                ld (remain),hl
                ld a,(slen+2)
                ld (remain+2),a
                ld hl,0
                ld (bufcnt),hl
                ld hl,z80h
                ld de,z80h+1
                ld bc,95
                ld (hl),0
                ldir
                ld hl,z80h
                ld de,30
                call get_bytes
                jp c,load_fail
                ld hl,(z80h+6)
                ld a,h
                or l
                jp nz,load_bad
                ld hl,z80h+30
                ld de,2
                call get_bytes
                jp c,load_fail
                ld hl,(z80h+30)
                ld a,h
                or a
                jp nz,load_bad
                ld a,l
                cp 65
                jp nc,load_bad
                ld e,l
                ld d,0
                ld hl,z80h+32
                call get_bytes
                jp c,load_fail
                call load_model
                ret c
dl_blk:         ld hl,(remain)
                ld a,(remain+2)
                or h
                or l
                ld hl,(bufcnt)
                or h
                or l
                jr z,dl_regs
                call get_byte
                jp c,load_fail
                ld (clen),a
                call get_byte
                jp c,load_fail
                ld (clen+1),a
                call get_byte
                jp c,load_fail
                call page_bank
                ld (blk),a
                cp #FF
                jr z,dl_skip
                call set_bank
                ld hl,(clen)
                ld a,h
                and l
                inc a
                jr nz,dl_packed
                ld hl,#C000
                ld de,16384
                call get_bytes
                jp c,load_fail
                jr dl_blk
dl_packed:      call unpack
                jp c,load_fail
                jr dl_blk
dl_skip:        ld hl,(clen)
                ld a,h
                and l
                inc a
                jr nz,dl_skip2
                ld hl,16384
dl_skip2:       ex de,hl
                call skip_stream
                jp c,load_fail
                jr dl_blk
dl_regs:        call z80_to_sb
                call write_sb
                call restore_ay
                call sess_load
                or a
                ret

sess_load:      ld hl,#FFFF
                ld (pk_game),hl
                ld hl,(slotbase)
                ld d,0
                ld a,CMD_READ
                call sd_io
                ret c
                call ptr0
                ld de,64
                call skip_data
                in a,(P_DATA)
                cp 'P'
                ret nz
                in a,(P_DATA)
                cp 'K'
                ret nz
                ld hl,pk_sess
                ld c,P_DATA
                ld b,0
                inir
                ld b,SESS_LEN-256
                inir
                ret
load_bad:       ld a,M_NOLOAD
                ld (msg),a
                scf
                ret
load_fail:      ld a,M_NOLOAD
                ld (msg),a
                scf
                ret

load_model:     ld a,(z80h+34)
                ld c,a
                ld b,F_48
                or a
                jr z,lm_got
                cp 1
                jr z,lm_got
                cp 3
                jr nz,lm_n3
                ld a,(z80h+30)
                cp 23
                jr nz,lm_got
                ld b,F_128
                jr lm_got
lm_n3:          ld b,F_128
                cp 4
                jr z,lm_got
                cp 5
                jr z,lm_got
                cp 6
                jr z,lm_got
                cp 9
                jr z,lm_got
                cp 12
                jr z,lm_got
                ld b,F_P3
                cp 7
                jr z,lm_got
                cp 8
                jr z,lm_got
                cp 13
                jr z,lm_got
                ld b,F_SCORP
                cp 10
                jr z,lm_got
                ld a,M_NOLOAD
                jr lm_no
lm_got:         ld a,b
                ld (family),a
                ld a,(sb+30)
                rrca
                rrca
                and 7
                ld c,a
                ld a,(z80h+35)
                ld (new7ffd),a
                ld a,(sb+29)
                ld (new1ffd),a
                ld a,b
                cp F_48
                jr nz,lm_not48
                ld a,c
                cp 3
                ret z
                ld a,#30
                ld (new7ffd),a
                ld a,c
                or a
                ret z
                cp 4
                ld a,M_NEED48
                jr nz,lm_no
                ld a,#04
                ld (new1ffd),a
                or a
                ret
lm_not48:       cp F_128
                jr nz,lm_not128
                ld a,c
                or a
                ret z
                ld a,M_NEED128
                jr lm_no
lm_not128:      ld e,a
                ld a,(z80h+30)
                cp 55
                ld a,0
                jr c,lm_no1ffd
                ld a,(z80h+86)
lm_no1ffd:      ld (new1ffd),a
                ld a,e
                cp F_P3
                jr nz,lm_scorp
                ld a,c
                cp 4
                ret z
                ld a,M_NEEDP3
                jr lm_no
lm_scorp:       ld a,c
                cp 5
                ret z
                ld a,M_NEEDSCORP
lm_no:          ld (msg),a
                scf
                ret

page_bank:      ld c,a
                ld a,(family)
                or a
                jr nz,pb_128
                ld a,c
                cp 4
                ld a,2
                ret z
                ld a,c
                cp 5
                ld a,0
                ret z
                ld a,c
                cp 8
                ld a,5
                ret z
                ld a,#FF
                ret
pb_128:         ld a,c
                sub 3
                jr c,pb_none
                ld c,a
                ld a,(family)
                cp F_SCORP
                ld a,c
                jr z,pb_16
                cp 8
                ret c
                jr pb_none
pb_16:          cp 16
                ret c
pb_none:        ld a,#FF
                ret

unpack:         ld hl,#C000
                xor a
                ld (pending),a
up_next:        ld a,(pending)
                or a
                jr nz,up_go
                ld de,(clen)
                ld a,d
                or e
                ret z
up_go:          call up_get
                ret c
                cp #ED
                jr nz,up_put
                ld de,(clen)
                ld a,d
                or e
                ld a,#ED
                jr z,up_put
                call up_get
                ret c
                cp #ED
                jr z,up_run
                ld (pendv),a
                ld a,1
                ld (pending),a
                ld a,#ED
                jr up_put
up_run:         call up_get
                ret c
                ld b,a
                call up_get
                ret c
                inc b
                dec b
                jr z,up_next
up_fill:        ld (hl),a
                inc hl
                djnz up_fill
                jr up_next
up_put:         ld (hl),a
                inc hl
                jr up_next

up_get:         ld a,(pending)
                or a
                jr z,ug_stream
                xor a
                ld (pending),a
                ld a,(pendv)
                ret
ug_stream:      push hl
                ld hl,(clen)
                dec hl
                ld (clen),hl
                pop hl
                jp get_byte

z80_to_sb:      ld hl,z80map
zs_map:         ld a,(hl)
                cp #FF
                jr z,zs_fix
                ld e,a
                ld d,0
                inc hl
                ld a,(hl)
                inc hl
                push hl
                ld hl,z80h
                add hl,de
                ld e,(hl)
                ld l,a
                ld h,0
                ld bc,sb
                add hl,bc
                ld (hl),e
                pop hl
                jr zs_map
zs_fix:         ld a,(z80h+12)
                ld b,a
                and 1
                rrca
                ld c,a
                ld a,(z80h+11)
                and #7F
                or c
                ld (sb+25),a
                ld a,b
                rrca
                and 7
                ld (sb+27),a
                ld a,(z80h+29)
                and 3
                ld c,a
                ld a,(z80h+27)
                or a
                jr z,zs_iff1
                set 2,c
zs_iff1:        ld a,(z80h+28)
                or a
                jr z,zs_iff2
                set 3,c
zs_iff2:        ld a,c
                ld (sb+26),a
                ld a,(new7ffd)
                ld (sb+28),a
                ld a,(new1ffd)
                ld (sb+29),a
                ret

restore_ay:     ld bc,#FFFD
                ld a,#FE
                out (c),a
                ld e,0
ra_clear:       ld bc,#FFFD
                out (c),e
                ld b,#BF
                xor a
                out (c),a
                inc e
                ld a,e
                cp 16
                jr nz,ra_clear
                ld bc,#FFFD
                ld a,#FF
                out (c),a
                ld hl,z80h+39
                ld e,0
ra_set:         ld bc,#FFFD
                out (c),e
                ld b,#BF
                ld a,(hl)
                out (c),a
                inc hl
                inc e
                ld a,e
                cp 16
                jr nz,ra_set
                ld a,(z80h+38)
                and #0F
                ld bc,#FFFD
                out (c),a
                ret

read_sb:        ld bc,P_SBIDX
                xor a
                out (c),a
                ld bc,P_SB
                ld hl,sb
                ld e,64
rs_l:           in a,(c)
                ld (hl),a
                inc hl
                dec e
                jr nz,rs_l
                ret

write_sb:       ld bc,P_SBIDX
                xor a
                out (c),a
                ld bc,P_SB
                ld hl,sb
                ld e,30
ws_l:           ld a,(hl)
                out (c),a
                inc hl
                dec e
                jr nz,ws_l
                ret

slot_lba:       ld a,(slot)
                ld l,a
                ld h,0
                add hl,hl
                add hl,hl
                add hl,hl
                add hl,hl
                add hl,hl
                add hl,hl
                ld d,h
                ld e,l
                add hl,hl
                add hl,hl
                add hl,hl
                add hl,de
                ld de,FIRST_LBA
                add hl,de
                ret

put_byte:       out (P_DATA),a
                push hl
                ld hl,(bufcnt)
                inc hl
                ld (bufcnt),hl
                ld a,h
                pop hl
                cp #40
                jr z,flush_full
                or a
                ret

put_bytes:      ld a,d
                or e
                ret z
                push hl
                ld hl,#4000
                ld bc,(bufcnt)
                or a
                sbc hl,bc
                ld bc,256
                call min_hl_bc
                ld b,d
                ld c,e
                call min_hl_bc
                ld b,h
                ld c,l
                ld hl,(bufcnt)
                add hl,bc
                ld (bufcnt),hl
                ex de,hl
                or a
                sbc hl,bc
                ex de,hl
                pop hl
                ld b,c
                ld c,P_DATA
                otir
                ld a,(bufcnt+1)
                cp #40
                jr nz,put_bytes
                call flush_full
                ret c
                jr put_bytes

flush_full:     push hl
                push de
                ld hl,(lba)
                ld d,31
                ld a,CMD_WRITE
                call sd_io
                jr c,ff_end
                ld hl,(lba)
                ld de,32
                add hl,de
                ld (lba),hl
                ld hl,0
                ld (bufcnt),hl
                call ptr0
                or a
ff_end:         pop de
                pop hl
                ret

flush_last:     ld hl,(bufcnt)
                ld a,h
                or l
                ret z
fl_pad:         ld a,h
                and 1
                or l
                jr z,fl_go
                xor a
                out (P_DATA),a
                inc hl
                jr fl_pad
fl_go:          ld a,h
                srl a
                dec a
                ld d,a
                ld hl,(lba)
                ld a,CMD_WRITE
                jp sd_io

get_byte:       push hl
                ld hl,(bufcnt)
                ld a,h
                or l
                jr nz,gb_have
                call refill
                jr c,gb_err
                ld hl,(bufcnt)
gb_have:        dec hl
                ld (bufcnt),hl
                pop hl
                in a,(P_DATA)
                or a
                ret
gb_err:         pop hl
                ret

get_bytes:      ld a,d
                or e
                ret z
                push hl
                ld hl,(bufcnt)
                ld a,h
                or l
                jr nz,gs_have
                call refill
                jr c,gs_err
                ld hl,(bufcnt)
gs_have:        ld bc,256
                call min_hl_bc
                ld b,d
                ld c,e
                call min_hl_bc
                ld b,h
                ld c,l
                ld hl,(bufcnt)
                or a
                sbc hl,bc
                ld (bufcnt),hl
                ex de,hl
                or a
                sbc hl,bc
                ex de,hl
                pop hl
                ld b,c
                ld c,P_DATA
                inir
                jr get_bytes
gs_err:         pop hl
                ret

skip_stream:    ld a,d
                or e
                ret z
                call get_byte
                ret c
                dec de
                jr skip_stream

refill:         push de
                push bc
                ld a,(remain+2)
                or a
                jr nz,rf_full
                ld hl,(remain)
                ld a,h
                cp #40
                jr nc,rf_full
                or l
                jr z,rf_eof
                jr rf_n
rf_full:        ld hl,#4000
rf_n:           ld (bufcnt),hl
                ex de,hl
                ld hl,(remain)
                or a
                sbc hl,de
                ld (remain),hl
                ld a,(remain+2)
                sbc a,0
                ld (remain+2),a
                ld hl,(bufcnt)
                ld de,511
                add hl,de
                ld a,h
                srl a
                dec a
                ld d,a
                ld hl,(lba)
                ld a,CMD_READ
                call sd_io
                jr c,rf_end
                ld hl,(lba)
                ld de,32
                add hl,de
                ld (lba),hl
                call ptr0
                or a
rf_end:         pop bc
                pop de
                ret
rf_eof:         scf
                jr rf_end

                include "lib.asm"

disk_hdr:       db "ZXSTATES", 1, 0
                dw 64, SLOT_SECS, FIRST_LBA
t_zxss:         db "ZXSS"

t_m48:          db "  48K"
t_m128:         db " 128K"
t_mpent:        db " PENT"
t_mp3:          db "   +3"
t_mscorp:       db "SCORP"
t_mnone:        db "     "

t_title:        db "SAVESTATES                      "
t_nodisk:       db " NO SAVESTATE DISK INSERTED     "
t_baddisk:      db " NOT A SAVESTATE DISK           "
t_anykey:       db " PRESS A KEY                    "
t_empty:        db " EMPTY SLOT                     "
t_keys:         db "<> SLOT S SAVE L LOAD N NAME ESC"
t_joy:          db " <> SLOT ^ LOAD v SAVE FIRE NAME"
t_keys_p:       db "<> S SAVE L LOAD N NAME P POKES "
t_joy_p:        db " <> ^ LOAD v SAVE F NAME F2 POKE"
t_ed_key:       db " TYPE A NAME  ENTER OK  ESC     "
t_ed_joy:       db " ^v LETTER  <> MOVE  FIRE OK    "
t_ovr_key:      db " OVERWRITE? PRESS S AGAIN       "
t_ovr_joy:      db " OVERWRITE? PRESS DOWN AGAIN    "
t_ld_key:       db " LOAD IT? PRESS L AGAIN         "
t_ld_joy:       db " LOAD IT? PRESS UP AGAIN        "
t_saving:       db " SAVING...                      "
t_loading:      db " LOADING...                     "
t_m_empty:      db " SLOT IS EMPTY                  "
t_m_nosave:     db " COULD NOT SAVE                 "
t_m_noload:     db " COULD NOT LOAD                 "
t_m_1024:       db " NOT IN THE 1024K MODES         "
t_m_128:        db " SWITCH TO 128K FIRST           "
t_m_p3:         db " SWITCH TO +3 FIRST             "
t_m_scorp:      db " SWITCH TO SCORPION FIRST       "
t_m_48:         db " SWITCH TO 48K OR 128K FIRST    "
t_m_ro:         db " THE DISK IS READ-ONLY          "
t_m_noname:     db " COULD NOT RENAME               "
t_m_nopokes:    db " NO POKES ON THIS DISK          "
t_zxpokes:      db "ZXPOKES", 1

msg_table:      dw t_m_empty, t_m_nosave, t_m_noload, t_m_1024, t_m_128
                dw t_m_p3, t_m_scorp, t_m_48, t_m_ro, t_m_noname
                dw t_m_nopokes

rom_end:
