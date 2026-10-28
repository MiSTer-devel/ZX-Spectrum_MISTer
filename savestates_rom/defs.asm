P_STAT          equ #000B
P_LBA0          equ #010B
P_LBA1          equ #020B
P_LBA2          equ #030B
P_LBA3          equ #040B
P_COUNT         equ #050B
P_PTR           equ #060B
P_BANK          equ #070B
P_SBIDX         equ #080B
P_SB            equ #090B
P_LEAVE         equ #0A0B
P_INPUT         equ #0B0B
P_FRAME         equ #0C0B
P_BORDER        equ #0D0B
P_ROMPG         equ #0E0B
P_LOADS         equ #0E0B
P_GAME          equ #0F0B
P_CRC           equ #100B
P_DATA          equ #0F

ST_BUSY         equ 0
ST_ERR          equ 1
ST_DISK         equ 2
ST_RO           equ 3

CMD_READ        equ 1
CMD_WRITE       equ 2

SCREEN          equ #2000
ATTRS           equ #3800
STACK           equ #4000

FIRST_LBA       equ 64
SLOT_SECS       equ 576
POKES_LBA       equ 1

A_BAR           equ #4F
A_NAME          equ #47
A_ASK           equ #56
A_EDIT          equ #78
A_CUR           equ #E8
A_GAME          equ #60

NAME_LEN        equ 30

EV_PREV         equ 0
EV_NEXT         equ 1
EV_SAVE         equ 2
EV_LOAD         equ 3
EV_BACK         equ 4
EV_NAME         equ 5
EV_POKES        equ 6

K_OK            equ 1
K_CANCEL        equ 2
K_LEFT          equ 3
K_RIGHT         equ 4
K_UP            equ 5
K_DOWN          equ 6
K_DEL           equ 7
K_ALT           equ 8

VARS            equ #3B00
sb              equ VARS
hdr             equ VARS+64
z80h            equ VARS+128
line            equ VARS+224
slot            equ VARS+256
nslots          equ VARS+257
valid           equ VARS+258
fmt             equ VARS+259
arch            equ VARS+260
slen            equ VARS+261
srtc            equ VARS+264
sborder         equ VARS+272
sname           equ VARS+273
ro              equ VARS+305
byjoy           equ VARS+306
confirm         equ VARS+307
msg             equ VARS+308
in_old          equ VARS+309
rep_cnt         equ VARS+310
lba             equ VARS+311
bufcnt          equ VARS+313
remain          equ VARS+315
hw              equ VARS+318
nblk            equ VARS+319
blk             equ VARS+320
slotbase        equ VARS+321
flen            equ VARS+323
family          equ VARS+326
new7ffd         equ VARS+327
new1ffd         equ VARS+328
scrbank         equ VARS+329
pending         equ VARS+330
pendv           equ VARS+331
clen            equ VARS+332
kold            equ VARS+336
know            equ VARS+344
jold            equ VARS+352
ecur            equ VARS+353
ename           equ VARS+354

pk_db           equ VARS+386
pk_count        equ VARS+387
pk_ixlba        equ VARS+389
pk_dtlba        equ VARS+391
pk_dtsecs       equ VARS+393
pk_id           equ VARS+395
pk_loads        equ VARS+399

pk_sess         equ VARS+400
pk_sid          equ pk_sess
pk_game         equ pk_sess+4
pk_on           equ pk_sess+6
pk_perm         equ pk_sess+22
pk_nund         equ pk_sess+38
pk_undo         equ pk_sess+39
UNDO_MAX        equ 64
SESS_LEN        equ 39 + UNDO_MAX*5

pk_str          equ pk_sess+SESS_LEN
pk_len          equ pk_str+20
pk_lf           equ pk_str+21
pk_lc           equ pk_str+27
pk_skip         equ pk_str+33
pk_vmin         equ pk_str+35
pk_vmax         equ pk_str+36
pk_val          equ pk_str+37
pk_fresh        equ pk_str+39
pk_msg          equ pk_str+40
pk_tmp          equ pk_str+42
gk_crc          equ pk_str+52
gk_known        equ pk_str+56
VARS_END        equ pk_str+57

F_48            equ 0
F_128           equ 1
F_P3            equ 2
F_SCORP         equ 3
