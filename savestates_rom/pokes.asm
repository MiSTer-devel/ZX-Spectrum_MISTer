                include "defs.asm"

A_ON            equ #44
ROWS            equ 20
QUERY_MAX       equ 20

                include "frame.asm"

main:           xor a
                ld bc,P_BORDER
                out (c),a
                ld hl,0
                ld (pk_msg),hl
                call key_sync
                call sess_ok
                jp z,cheats
                ld hl,0
                jr find_at

find:           ld hl,(pk_game)
find_at:        ld (pk_lf),hl
                ld hl,(pk_count)
                ld (pk_lf+4),hl
                xor a
                ld (pk_len),a
                ld ix,pk_lf
                call lst_center
                call cls
                ld hl,t_find
                ld b,0
                ld c,A_BAR
                call print_row
fi_draw:        call draw_games
                call draw_query
                call find_help
fi_wait:        call get_key
                or a
                jr z,fi_wait
                cp K_CANCEL
                jr z,fi_back
                cp K_ALT
                jr z,fi_back
                cp K_OK
                jr z,fi_ok
                cp K_DEL
                jr z,fi_del
                ld ix,pk_lf
                call lst_key
                jr z,fi_draw
                cp ' '
                jr c,fi_wait
                cp 127
                jr nc,fi_wait
                call upper
                ld e,a
                ld a,(pk_len)
                cp QUERY_MAX
                jr nc,fi_wait
                ld c,a
                ld b,0
                ld hl,pk_str
                add hl,bc
                ld (hl),e
                inc a
                ld (pk_len),a
fi_look:        call search
                jr fi_draw
fi_del:         ld a,(pk_len)
                or a
                jr z,fi_wait
                dec a
                ld (pk_len),a
                jr fi_look
fi_back:        call sess_ok
                jp z,cheats
                xor a
                jp to_page

fi_ok:          call sess_ok
                ld hl,(pk_lf)
                jr nz,fi_new
                ld de,(pk_game)
                call cmp_hl_de
                jp z,cheats
fi_new:         ld (pk_game),hl
                ld hl,pk_id
                ld de,pk_sid
                ld bc,4
                ldir
                ld hl,pk_on
                ld b,33
fi_clr:         ld (hl),0
                inc hl
                djnz fi_clr
                jp cheats

search:         ld hl,0
                ld (pk_tmp),hl
                ld hl,(pk_count)
                ld (pk_tmp+2),hl
se_l:           ld hl,(pk_tmp+2)
                ld de,(pk_tmp)
                or a
                sbc hl,de
                jr z,se_end
                srl h
                rr l
                add hl,de
                push hl
                call rec_cmp
                pop hl
                jr nc,se_hi
                inc hl
                ld (pk_tmp),hl
                jr se_l
se_hi:          ld (pk_tmp+2),hl
                jr se_l
se_end:         ld hl,(pk_tmp)
                ld de,(pk_count)
                call cmp_hl_de
                jr c,se_set
                ex de,hl
                dec hl
se_set:         ld (pk_lf),hl
                ld ix,pk_lf
                jp lst_center

rec_cmp:        ld d,0
                call idx_seek
                jr nc,rc_go
                or 1
                ret
rc_go:          ld a,(pk_len)
                or a
                ret z
                ld b,a
                ld hl,pk_str
rc_l:           in a,(P_DATA)
                call upper
                cp (hl)
                ret nz
                inc hl
                djnz rc_l
                ret

idx_seek:       ld a,l
                and 15
                push af
                srl h
                rr l
                srl h
                rr l
                srl h
                rr l
                srl h
                rr l
                ld bc,(pk_ixlba)
                add hl,bc
                ld a,CMD_READ
                call sd_io
                pop bc
                ret c
                push bc
                call ptr0
                pop bc
                ld l,b
                ld h,0
                add hl,hl
                add hl,hl
                add hl,hl
                add hl,hl
                add hl,hl
                ex de,hl
                jp skip

draw_games:     ld hl,(pk_lf+2)
                ld d,2
                call idx_seek
                ld hl,(pk_lf+2)
                ld (pk_tmp),hl
                ld a,2
dg_row:         ld (pk_tmp+2),a
                call clear_line
                ld hl,(pk_tmp)
                ld de,(pk_count)
                call cmp_hl_de
                ld a,A_NAME
                jr nc,dg_print
                ld hl,line+1
                ld b,29
                ld c,P_DATA
                inir
                in a,(P_DATA)
                in a,(P_DATA)
                in a,(P_DATA)
                ld hl,(pk_tmp)
                ld de,(pk_lf)
                call cmp_hl_de
                ld a,A_NAME
                jr nz,dg_print
                ld a,A_EDIT
dg_print:       ld c,a
                ld a,(pk_tmp+2)
                ld b,a
                ld hl,line
                call print_row
                ld hl,(pk_tmp)
                inc hl
                ld (pk_tmp),hl
                ld a,(pk_tmp+2)
                inc a
                cp 2+ROWS
                jr nz,dg_row
                ret

draw_query:     call clear_line
                ld hl,t_name
                ld de,line
                ld bc,7
                ldir
                ld a,(pk_len)
                or a
                jr z,dq_cur
                ld c,a
                ld b,0
                ld hl,pk_str
                ldir
dq_cur:         ld a,'_'
                ld (de),a
                ld hl,(pk_lf)
                call rec_cmp
                ld c,A_NAME
                jr z,dq_print
                ld c,A_ASK
dq_print:       ld hl,line
                ld b,22
                jp print_row

find_help:      ld hl,t_fkeys
                ld a,(byjoy)
                or a
                jr z,help
                ld hl,t_fjoy
help:           ld b,23
                ld c,A_BAR
                jp print_row

lst_key:        ld l,(ix+0)
                ld h,(ix+1)
                cp K_UP
                jr nz,lk_1
                ld a,h
                or l
                ret z
                dec hl
                jr lk_set
lk_1:           cp K_DOWN
                jr nz,lk_2
                inc hl
                jr lk_set
lk_2:           cp K_LEFT
                jr nz,lk_3
                ld de,-ROWS
                add hl,de
                jr c,lk_set
                ld hl,0
                jr lk_set
lk_3:           cp K_RIGHT
                ret nz
                ld de,ROWS
                add hl,de
lk_set:         ld (ix+0),l
                ld (ix+1),h
                call lst_fix
                xor a
                ret

lst_center:     ld l,(ix+0)
                ld h,(ix+1)
                ld de,-9
                add hl,de
                jr c,lc_set
                ld hl,0
lc_set:         ld (ix+2),l
                ld (ix+3),h

lst_fix:        ld e,(ix+0)
                ld d,(ix+1)
                ld l,(ix+4)
                ld h,(ix+5)
                dec hl
                call cmp_hl_de
                jr nc,lf_1
                ex de,hl
                ld (ix+0),e
                ld (ix+1),d
lf_1:           ld l,(ix+2)
                ld h,(ix+3)
                call cmp_hl_de
                jr c,lf_2
                ex de,hl
                jr lf_top
lf_2:           ld bc,ROWS-1
                add hl,bc
                call cmp_hl_de
                jr nc,lf_3
                ex de,hl
                or a
                sbc hl,bc
                jr lf_top
lf_3:           ld l,(ix+2)
                ld h,(ix+3)
lf_top:         ex de,hl
                ld l,(ix+4)
                ld h,(ix+5)
                ld bc,ROWS
                or a
                sbc hl,bc
                jr nc,lf_4
                ld hl,0
lf_4:           call cmp_hl_de
                jr c,lf_5
                ex de,hl
lf_5:           ld (ix+2),l
                ld (ix+3),h
                ret

cmp_hl_de:      or a
                sbc hl,de
                add hl,de
                ret

cheats:         call load_game
                jr c,ch_fail
                ld hl,0
                ld (pk_lc),hl
                ld (pk_lc+2),hl
                call cls
                call draw_game
ch_draw:        call draw_cheats
ch_info:        call draw_info
                call cheats_help
ch_wait:        call get_key
                or a
                jr z,ch_wait
                ld hl,0
                ld (pk_msg),hl
                cp K_CANCEL
                jr z,ch_back
                cp K_ALT
                jp z,find
                cp 'f'
                jp z,find
                cp 'F'
                jp z,find
                cp K_OK
                jr z,ch_toggle
                ld ix,pk_lc
                call lst_key
                jr z,ch_draw
                jr ch_info
ch_back:        xor a
                jp to_page
ch_toggle:      ld a,(pk_lc)
                call toggle
                jr ch_draw

ch_fail:        call cls
                ld hl,t_noread
                ld b,22
                ld c,A_ASK
                call print_row
cf_wait:        call get_key
                or a
                jr z,cf_wait
                xor a
                jp to_page

load_game:      ld hl,(pk_game)
                ld d,0
                call idx_seek
                ret c
                ld de,29
                call skip
                in a,(P_DATA)
                ld l,a
                in a,(P_DATA)
                ld h,a
                in a,(P_DATA)
                ld e,a
                push hl
                ld a,h
                and 1
                ld h,a
                ld (pk_skip),hl
                pop hl
                ld l,h
                ld h,e
                srl h
                rr l
                ex de,hl
                ld hl,(pk_dtsecs)
                or a
                sbc hl,de
                ld a,h
                or a
                jr nz,lg_32
                ld a,l
                cp 33
                jr c,lg_n
lg_32:          ld a,32
lg_n:           dec a
                ld hl,(pk_dtlba)
                add hl,de
                ld d,a
                ld a,CMD_READ
                call sd_io
                ret c
                call game_start
                in a,(P_DATA)
                ld e,a
                ld d,0
                call skip
                in a,(P_DATA)
                ld (pk_lc+4),a
                xor a
                ld (pk_lc+5),a
                ret

game_start:     call ptr0
                ld de,(pk_skip)

skip:           ld a,d
                or e
                ret z
                jp skip_data

get_text:       ld b,a
                cp c
                jr c,gt_1
                ld b,c
gt_1:           sub b
                push af
                ld a,b
                or a
                jr z,gt_2
                ld c,P_DATA
                inir
gt_2:           pop af
                ret

draw_game:      call game_start
                in a,(P_DATA)
                ld (pk_tmp),a
                call clear_line
                ld a,(pk_tmp)
                ld c,31
                ld hl,line+1
                call get_text
                ld (pk_tmp),a
                ld hl,line
                ld b,0
                ld c,A_BAR
                call print_row
                call clear_line
                ld a,(pk_tmp)
                ld c,31
                ld hl,line+1
                call get_text
                ld hl,line
                ld b,1
                ld c,A_BAR
                jp print_row

cheat_seek:     ld (pk_tmp+3),a
                call game_start
                in a,(P_DATA)
                ld e,a
                ld d,0
                call skip
                in a,(P_DATA)
                ld a,(pk_tmp+3)
                or a
                ret z
                ld b,a
cs_l:           push bc
                in a,(P_DATA)
                ld e,a
                ld d,0
                call skip
                call skip_pokes
                pop bc
                djnz cs_l
                ret

skip_pokes:     in a,(P_DATA)
                in a,(P_DATA)
                in a,(P_DATA)
                ld l,a
                in a,(P_DATA)
                ld h,a
                ld d,h
                ld e,l
                add hl,hl
                add hl,hl
                add hl,de
                ex de,hl
                jp skip

draw_cheats:    ld a,(pk_lc+2)
                call cheat_seek
                ld a,(pk_lc+2)
                ld (pk_tmp+1),a
                ld a,2
dc_row:         ld (pk_tmp+2),a
                call clear_line
                ld a,(pk_tmp+1)
                ld hl,pk_lc+4
                cp (hl)
                ld a,A_NAME
                jr nc,dc_print
                in a,(P_DATA)
                ld c,28
                ld hl,line+3
                call get_text
                ld e,a
                ld d,0
                call skip
                call skip_pokes
                ld a,(pk_tmp+1)
                call bit_on
                ld a,A_NAME
                jr z,dc_cur
                ld a,'*'
                ld (line+1),a
                ld a,(pk_tmp+1)
                call bit_perm
                jr z,dc_on
                ld a,'+'
                ld (line+1),a
dc_on:          ld a,A_ON
dc_cur:         ld c,a
                ld a,(pk_lc)
                ld hl,pk_tmp+1
                cp (hl)
                ld a,c
                jr nz,dc_print
                ld a,A_EDIT
dc_print:       ld c,a
                ld a,(pk_tmp+2)
                ld b,a
                ld hl,line
                call print_row
                ld hl,pk_tmp+1
                inc (hl)
                ld a,(pk_tmp+2)
                inc a
                cp 2+ROWS
                jr nz,dc_row
                ret

draw_info:      ld hl,(pk_msg)
                ld a,h
                or l
                jr z,di_name
                ld b,22
                ld c,A_ASK
                jp print_row
di_name:        call clear_line
                ld a,(pk_lc)
                call cheat_seek
                in a,(P_DATA)
                cp 29
                jr c,di_print
                push af
                ld de,28
                call skip
                pop af
                sub 28
                ld c,30
                ld hl,line+1
                call get_text
di_print:       ld hl,line
                ld b,22
                ld c,A_NAME
                jp print_row

cheats_help:    ld hl,t_ckeys
                ld a,(byjoy)
                or a
                jp z,help
                ld hl,t_cjoy
                jp help

bit_on:         ld hl,pk_on
                call bit_of
                and (hl)
                ret

bit_perm:       ld hl,pk_perm
                call bit_of
                and (hl)
                ret

bit_of:         ld c,a
                rrca
                rrca
                rrca
                and 31
                ld e,a
                ld d,0
                add hl,de
                ld a,c
                and 7
                ld b,a
                ld a,1
                ret z
bo_l:           rlca
                djnz bo_l
                ret

toggle:         ld (pk_tmp+4),a
                call bit_on
                jr z,tg_on
                ld a,(pk_tmp+4)
                call bit_perm
                jr z,tg_off
                ld hl,t_cant
                ld (pk_msg),hl
                ret
tg_off:         call undo
                ld a,(pk_tmp+4)
                ld hl,pk_on
                call bit_of
                cpl
                and (hl)
                ld (hl),a
                ret

tg_on:          ld a,(pk_tmp+4)
                call cheat_seek
                in a,(P_DATA)
                ld e,a
                ld d,0
                call skip
                in a,(P_DATA)
                ld (pk_vmin),a
                in a,(P_DATA)
                ld (pk_vmax),a
                in a,(P_DATA)
                ld l,a
                in a,(P_DATA)
                ld h,a
                ld (pk_tmp+5),hl
                xor a
                ld (pk_tmp+7),a
tg_scan:        ld a,h
                or l
                jr z,tg_asks
                in a,(P_DATA)
                and #80
                ld e,a
                ld a,(pk_tmp+7)
                or e
                ld (pk_tmp+7),a
                in a,(P_DATA)
                in a,(P_DATA)
                in a,(P_DATA)
                in a,(P_DATA)
                dec hl
                jr tg_scan
tg_asks:        ld a,(pk_tmp+7)
                or a
                jr z,tg_room
                call ask_value
                ret c
tg_room:
                ld a,(pk_nund)
                ld e,a
                ld a,UNDO_MAX
                sub e
                ld l,a
                ld h,0
                ld de,(pk_tmp+5)
                call cmp_hl_de
                ld a,0
                jr nc,tg_fit
                inc a
tg_fit:         ld (pk_tmp+8),a
                call apply
                ld a,(pk_tmp+4)
                ld hl,pk_on
                call bit_of
                or (hl)
                ld (hl),a
                ld a,(pk_tmp+8)
                or a
                ret z
                ld a,(pk_tmp+4)
                ld hl,pk_perm
                call bit_of
                or (hl)
                ld (hl),a
                ld hl,t_onperm
                ld (pk_msg),hl
                ret

apply:          ld a,(pk_tmp+4)
                call cheat_seek
                in a,(P_DATA)
                ld e,a
                ld d,0
                call skip
                in a,(P_DATA)
                in a,(P_DATA)
                in a,(P_DATA)
                ld l,a
                in a,(P_DATA)
                ld h,a
                ld (pk_tmp+5),hl
ap_l:           ld hl,(pk_tmp+5)
                ld a,h
                or l
                ret z
                dec hl
                ld (pk_tmp+5),hl
                in a,(P_DATA)
                ld (pk_tmp),a
                in a,(P_DATA)
                ld (pk_tmp+1),a
                in a,(P_DATA)
                ld (pk_tmp+2),a
                in a,(P_DATA)
                ld (pk_tmp+9),a
                in a,(P_DATA)
                ld a,(pk_tmp)
                rlca
                jr nc,ap_where
                ld a,(pk_val)
                ld (pk_tmp+9),a
ap_where:       call poke_where
                jr c,ap_l
                push hl
                push af
                call set_bank
                pop af
                pop hl
                ld e,a
                ld a,(pk_tmp+8)
                or a
                jr nz,ap_put
                push hl
                push de
                ld a,(pk_nund)
                ld c,a
                ld b,0
                ld h,b
                ld l,c
                add hl,hl
                add hl,hl
                add hl,bc
                ld bc,pk_undo
                add hl,bc
                pop de
                pop bc
                ld a,(pk_tmp+4)
                ld (hl),a
                inc hl
                ld (hl),e
                inc hl
                ld (hl),c
                inc hl
                ld (hl),b
                inc hl
                ld a,(bc)
                ld (hl),a
                ld hl,pk_nund
                inc (hl)
                ld h,b
                ld l,c
ap_put:         ld a,(pk_tmp+9)
                ld (hl),a
                jr ap_l

poke_where:     ld a,(pk_tmp)
                and 15
                cp 8
                jr c,pw_win
                ld a,(pk_tmp+2)
                rlca
                rlca
                and 3
                ld e,a
                ld a,(sb+30)
                rrca
                rrca
                and 7
                ld d,a
                cp 4
                jr nz,pw_norm
                ld a,(sb+29)
                bit 0,a
                jr z,pw_norm
                rrca
                and 3
                add a,a
                add a,a
                add a,e
                ld e,a
                ld d,0
                ld hl,all_ram
                add hl,de
                ld a,(hl)
                jr pw_win
pw_norm:        ld a,e
                or a
                jr nz,pw_ram
                scf
                ret
pw_ram:         cp 3
                jr z,pw_top
                cp 1
                ld a,5
                jr z,pw_win
                ld a,2
                jr pw_win
pw_top:         ld a,d
                cp 3
                ld a,0
                jr z,pw_win
                ld a,(sb+28)
                and 7
                ld e,a
                ld a,d
                cp 5
                ld a,e
                jr nz,pw_win
                ld a,(sb+29)
                and #10
                rrca
                or e
pw_win:         ld e,a
                ld hl,(pk_tmp+1)
                ld a,h
                or #C0
                ld h,a
                ld a,e
                or a
                ret

all_ram:        db 0, 1, 2, 3
                db 4, 5, 6, 7
                db 4, 5, 6, 3
                db 4, 7, 6, 3

undo:           ld a,(pk_nund)
                or a
                ret z
                ld b,a
                dec a
                ld l,a
                ld h,0
                ld d,h
                ld e,l
                add hl,hl
                add hl,hl
                add hl,de
                ld de,pk_undo
                add hl,de
un_l:           push bc
                push hl
                ld a,(pk_tmp+4)
                cp (hl)
                jr nz,un_next
                inc hl
                ld a,(hl)
                push hl
                call set_bank
                pop hl
                inc hl
                ld e,(hl)
                inc hl
                ld d,(hl)
                inc hl
                ld a,(hl)
                ld (de),a
un_next:        pop hl
                ld de,-5
                add hl,de
                pop bc
                djnz un_l
                ld hl,pk_undo
                ld de,pk_undo
                ld a,(pk_nund)
                ld b,a
                ld c,0
un_c:           ld a,(pk_tmp+4)
                cp (hl)
                jr z,un_drop
                push bc
                ld bc,5
                ldir
                pop bc
                inc c
                djnz un_c
                jr un_end
un_drop:        push bc
                ld bc,5
                add hl,bc
                pop bc
                djnz un_c
un_end:         ld a,c
                ld (pk_nund),a
                ret

ask_value:      ld a,(pk_vmax)
                ld l,a
                ld h,0
                ld (pk_val),hl
                ld a,1
                ld (pk_fresh),a
av_draw:        call clear_line
                ld hl,t_value
                ld de,line
                ld bc,7
                ldir
                ld a,(pk_vmin)
                ld e,a
                ld d,0
                ld hl,line+7
                call num
                ld (hl),'-'
                inc hl
                ld a,(pk_vmax)
                ld e,a
                ld d,0
                call num
                ld (hl),':'
                inc hl
                inc hl
                ld de,(pk_val)
                call num
                ld (hl),'_'
                ld hl,line
                ld b,22
                ld c,A_EDIT
                call print_row
                ld hl,t_vkeys
                ld a,(byjoy)
                or a
                call z,help
                ld hl,t_vjoy
                ld a,(byjoy)
                or a
                call nz,help
av_key:         call get_key
                or a
                jr z,av_key
                cp K_CANCEL
                jp z,av_no
                cp K_ALT
                jp z,av_no
                cp K_OK
                jp z,av_ok
                ld de,1
                cp K_UP
                jr z,av_add
                ld de,-1
                cp K_DOWN
                jr z,av_add
                ld de,10
                cp K_RIGHT
                jr z,av_add
                ld de,-10
                cp K_LEFT
                jr z,av_add
                cp K_DEL
                jr z,av_del
                sub '0'
                cp 10
                jr nc,av_key
                ld e,a
                ld d,0
                ld hl,0
                ld a,(pk_fresh)
                or a
                jr nz,av_dig
                ld hl,(pk_val)
av_dig:         xor a
                ld (pk_fresh),a
                push de
                ld d,h
                ld e,l
                add hl,hl
                add hl,hl
                add hl,de
                add hl,hl
                pop de
                add hl,de
                ld de,1000
                call cmp_hl_de
                jr nc,av_key
                ld (pk_val),hl
                jp av_draw
av_del:         xor a
                ld (pk_fresh),a
                ld hl,(pk_val)
                ld bc,10
                call nm_div
                ld l,a
                ld h,0
                ld (pk_val),hl
                jp av_draw
av_add:         xor a
                ld (pk_fresh),a
                ld hl,(pk_val)
                add hl,de
                bit 7,h
                jr nz,av_min
                ld a,(pk_vmax)
                ld e,a
                ld d,0
                call cmp_hl_de
                jr nc,av_max
                ld a,(pk_vmin)
                ld e,a
                call cmp_hl_de
                jr nc,av_set
av_min:         ld a,(pk_vmin)
                ld l,a
                ld h,0
                jr av_set
av_max:         ex de,hl
av_set:         ld (pk_val),hl
                jp av_draw
av_ok:          ld hl,(pk_val)
                ld a,(pk_vmax)
                ld e,a
                ld d,0
                call cmp_hl_de
                jr z,av_yes
                jr nc,av_max
                ld a,(pk_vmin)
                ld e,a
                call cmp_hl_de
                jr c,av_min
av_yes:         or a
                ret
av_no:          scf
                ret

num:            ex de,hl
                push de
                ld bc,100
                call nm_div
                ld d,a
                ld bc,10
                call nm_div
                ld e,a
                ld c,l
                pop hl
                ld a,d
                or a
                jr z,nm_1
                add a,'0'
                ld (hl),a
                inc hl
                ld a,e
                jr nm_t
nm_1:           ld a,e
                or a
                jr z,nm_u
nm_t:           add a,'0'
                ld (hl),a
                inc hl
nm_u:           ld a,c
                add a,'0'
                ld (hl),a
                inc hl
                ret

nm_div:         ld a,#FF
nd_l:           inc a
                or a
                sbc hl,bc
                jr nc,nd_l
                add hl,bc
                ret

upper:          cp 'a'
                ret c
                cp 'z'+1
                ret nc
                sub 32
                ret

                include "lib.asm"

t_find:         db " POKES: FIND THE GAME           "
t_name:         db " NAME: "
t_fkeys:        db " TYPE A NAME ^v<> MOVE ENTER OK "
t_fjoy:         db " ^v<> MOVE  FIRE OK  F2 BACK    "
t_ckeys:        db " ^v MOVE ENTER ON/OFF F FIND ESC"
t_cjoy:         db " ^v MOVE  FIRE ON/OFF  F2 FIND  "
t_value:        db " VALUE "
t_vkeys:        db " TYPE IT  ^v<> CHANGE  ENTER OK "
t_vjoy:         db " ^v<> CHANGE  FIRE OK           "
t_cant:         db " THIS CHEAT CAN'T BE TURNED OFF "
t_onperm:       db " ON FOR GOOD: TOO MANY POKES ON "
t_noread:       db " COULD NOT READ THE POKES       "

page1_end:
