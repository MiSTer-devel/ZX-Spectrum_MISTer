"""Builds the savestate device's hidden ROM: two 8 KB pages, both at #0000,
picked with port #0E0B.

    python tools/savestates_rom/build_rom.py

  page 0  savestates_rom/savestate_rom.asm   the slots
  page 1  savestates_rom/pokes.asm           the POKEs

Writes savestates_rom/savestate_rom.bin and .hex (16 KB),
rtl/savestate_rom.mif, and a listing and the symbols of each page:
savestates_rom/savestate_rom.lst and .sym, savestates_rom/pokes.lst and .sym.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import z80asm

PAGE = 8192
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def page(src, lst, sym):
    a = z80asm.Asm()
    try:
        a.assemble(os.path.join(ROOT, src))
    except z80asm.AsmError as e:
        sys.exit(str(e))
    if a.out and max(a.out) >= PAGE:
        sys.exit('%s goes past %04X' % (src, PAGE))
    open(os.path.join(ROOT, lst), 'w', newline='\n').write('\n'.join(a.listing) + '\n')
    open(os.path.join(ROOT, sym), 'w', newline='\n').write(
        ''.join('%s %04X\n' % (k, v & 0xFFFF) for k, v in sorted(a.sym.items())))
    print('%s: %d bytes used of %d' % (src, len(a.out), PAGE))
    return bytes(a.out.get(i, 0) for i in range(PAGE))


data = (page('savestates_rom/savestate_rom.asm', 'savestates_rom/savestate_rom.lst', 'savestates_rom/savestate_rom.sym') +
        page('savestates_rom/pokes.asm', 'savestates_rom/pokes.lst', 'savestates_rom/pokes.sym'))
z80asm.write_outputs(data, os.path.join(ROOT, 'savestates_rom/savestate_rom.bin'),
                     {'--hex': os.path.join(ROOT, 'savestates_rom/savestate_rom.hex'),
                      '--mif': os.path.join(ROOT, 'rtl/savestate_rom.mif')})
