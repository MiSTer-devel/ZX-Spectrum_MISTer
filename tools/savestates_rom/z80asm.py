"""Small two-pass Z80 assembler for the savestate ROM.

    python z80asm.py source.asm out.bin [--mif out.mif] [--hex out.hex]
                     [--size N] [--list out.lst]

Covers the documented Z80 instruction set with IX/IY, in the usual syntax
(pasmo, sjasmplus): labels with or without ':', EQU, ORG, DB/DEFB/DEFM,
DW/DEFW, DS/DEFS, INCBIN, INCLUDE, ';' comments. Numbers: 123, #7F, $7F, 0x7F, 7Fh,
%1010, 0b1010, 'A'. '$' alone is the current address.
"""
import os
import re
import sys


class AsmError(Exception):
    pass


R8 = {'b': 0, 'c': 1, 'd': 2, 'e': 3, 'h': 4, 'l': 5, 'a': 7}
RP = {'bc': 0, 'de': 1, 'hl': 2, 'sp': 3}
RP2 = {'bc': 0, 'de': 1, 'hl': 2, 'af': 3}
CC = {'nz': 0, 'z': 1, 'nc': 2, 'c': 3, 'po': 4, 'pe': 5, 'p': 6, 'm': 7}
ALU = {'add': 0, 'adc': 1, 'sub': 2, 'sbc': 3, 'and': 4, 'xor': 5, 'or': 6, 'cp': 7}
ROT = {'rlc': 0, 'rrc': 1, 'rl': 2, 'rr': 3, 'sla': 4, 'sra': 5, 'sll': 6, 'srl': 7}
IDX = {'ix': 0xDD, 'iy': 0xFD}
SIMPLE = {
    'nop': [0x00], 'halt': [0x76], 'di': [0xF3], 'ei': [0xFB], 'exx': [0xD9],
    'daa': [0x27], 'cpl': [0x2F], 'ccf': [0x3F], 'scf': [0x37],
    'rlca': [0x07], 'rla': [0x17], 'rrca': [0x0F], 'rra': [0x1F],
    'neg': [0xED, 0x44], 'reti': [0xED, 0x4D], 'retn': [0xED, 0x45],
    'rld': [0xED, 0x6F], 'rrd': [0xED, 0x67],
    'ldi': [0xED, 0xA0], 'ldir': [0xED, 0xB0], 'ldd': [0xED, 0xA8], 'lddr': [0xED, 0xB8],
    'cpi': [0xED, 0xA1], 'cpir': [0xED, 0xB1], 'cpd': [0xED, 0xA9], 'cpdr': [0xED, 0xB9],
    'ini': [0xED, 0xA2], 'inir': [0xED, 0xB2], 'ind': [0xED, 0xAA], 'indr': [0xED, 0xBA],
    'outi': [0xED, 0xA3], 'otir': [0xED, 0xB3], 'outd': [0xED, 0xAB], 'otdr': [0xED, 0xBB],
}

NUM = re.compile(r"(?<![\w$#%])(?:0x([0-9a-f]+)|[#$]([0-9a-f]+)|([0-9][0-9a-f]*)h|%([01]+)|0b([01]+)|(\d+))(?![\w])", re.I)


def split_args(s):
    out, depth, cur, q = [], 0, '', None
    for ch in s:
        if q:
            cur += ch
            if ch == q:
                q = None
        elif ch in '"\'':
            # af' is a register, not a string
            if ch == "'" and cur.strip().lower() == 'af':
                cur += ch
                continue
            q = ch
            cur += ch
        elif ch == '(':
            depth += 1
            cur += ch
        elif ch == ')':
            depth -= 1
            cur += ch
        elif ch == ',' and depth == 0:
            out.append(cur.strip())
            cur = ''
        else:
            cur += ch
    if cur.strip():
        out.append(cur.strip())
    return out


def wrapped(s):
    """'(...)' with the first bracket closing at the end."""
    if not (s.startswith('(') and s.endswith(')')):
        return False
    depth = 0
    for i, ch in enumerate(s):
        depth += ch == '('
        depth -= ch == ')'
        if depth == 0 and i != len(s) - 1:
            return False
    return True


class Asm:
    def __init__(self):
        self.sym = {}
        self.out = {}
        self.listing = []

    # expressions -------------------------------------------------------
    def value(self, expr, pc, final):
        e = expr.strip()
        e = re.sub(r"'(\\?.)'", lambda m: str(ord(m.group(1)[-1])), e)

        def num(m):
            g = m.groups()
            if g[0]: return str(int(g[0], 16))
            if g[1]: return str(int(g[1], 16))
            if g[2]: return str(int(g[2], 16))
            if g[3]: return str(int(g[3], 2))
            if g[4]: return str(int(g[4], 2))
            return g[5]
        e = NUM.sub(num, e)
        e = re.sub(r'(?<![\w])\$(?![\w])', str(pc), e)

        def name(m):
            n = m.group(0)
            if n.lower() in self.sym:
                return str(self.sym[n.lower()])
            if final:
                raise AsmError('unknown symbol %s' % n)
            return '0'
        e = re.sub(r'[A-Za-z_.][\w.]*', name, e)
        if not re.fullmatch(r'[\d\s+\-*/%()<>&|^~]*', e):
            raise AsmError('bad expression %r' % expr)
        e = e.replace('/', '//')
        try:
            return int(eval(e, {'__builtins__': {}}))
        except Exception:
            raise AsmError('bad expression %r' % expr)

    # operands ----------------------------------------------------------
    def operand(self, t):
        s = t.strip()
        low = s.lower().replace(' ', '')
        if low in R8:
            return ('r8', low)
        if low in ('i', 'r'):
            return ('ir', low)
        if low in ("af'",):
            return ('afp', low)
        if low in ('bc', 'de', 'hl', 'sp', 'af'):
            return ('r16', low)
        if low in IDX:
            return ('ix', low)
        if wrapped(s):
            inner = s[1:-1].strip()
            il = inner.lower().replace(' ', '')
            if il in ('hl', 'bc', 'de', 'sp', 'c'):
                return ('ind', il)
            m = re.fullmatch(r'(ix|iy)([+-].*)?', il)
            if m:
                d = inner.strip()[2:].strip()
                return ('idx', m.group(1), d if d else '0')
            return ('mem', inner)
        return ('imm', s)

    # one instruction ---------------------------------------------------
    def encode(self, mn, args, pc, final):
        v = lambda e: self.value(e, pc, final)
        ops = [self.operand(a) for a in args]
        n = len(ops)
        k = [o[0] for o in ops]

        def b8(x):
            if final and not -128 <= x <= 255:
                raise AsmError('byte out of range: %d' % x)
            return x & 0xFF

        def w16(x):
            if final and not -32768 <= x <= 65535:
                raise AsmError('word out of range: %d' % x)
            return [x & 0xFF, (x >> 8) & 0xFF]

        def disp(o):
            d = v(o[2])
            if final and not -128 <= d <= 127:
                raise AsmError('index out of range: %d' % d)
            return d & 0xFF

        def reg_or_mem(o):
            """r, (hl) or (ix+d): (prefix, code, disp)"""
            if o[0] == 'r8':
                return [], R8[o[1]], []
            if o[0] == 'ind' and o[1] == 'hl':
                return [], 6, []
            if o[0] == 'idx':
                return [IDX[o[1]]], 6, [disp(o)]
            raise AsmError('bad operand')

        if mn in SIMPLE and n == 0:
            return list(SIMPLE[mn])

        if mn == 'ld':
            a, b = ops
            # 8 bit
            if a[0] == 'r8' and b[0] == 'r8':
                return [0x40 | R8[a[1]] << 3 | R8[b[1]]]
            if a[0] == 'r8' and b[0] == 'imm':
                return [0x06 | R8[a[1]] << 3, b8(v(b[1]))]
            if a[0] == 'r8' and (b[0] == 'idx' or (b[0] == 'ind' and b[1] == 'hl')):
                p, _, d = reg_or_mem(b)
                return p + [0x46 | R8[a[1]] << 3] + d
            if b[0] == 'r8' and (a[0] == 'idx' or (a[0] == 'ind' and a[1] == 'hl')):
                p, _, d = reg_or_mem(a)
                return p + [0x70 | R8[b[1]]] + d
            if (a[0] == 'idx' or (a[0] == 'ind' and a[1] == 'hl')) and b[0] == 'imm':
                p, _, d = reg_or_mem(a)
                return p + [0x36] + d + [b8(v(b[1]))]
            if a == ('r8', 'a') and b[0] == 'ind' and b[1] in ('bc', 'de'):
                return [0x0A if b[1] == 'bc' else 0x1A]
            if b == ('r8', 'a') and a[0] == 'ind' and a[1] in ('bc', 'de'):
                return [0x02 if a[1] == 'bc' else 0x12]
            if a == ('r8', 'a') and b[0] == 'mem':
                return [0x3A] + w16(v(b[1]))
            if b == ('r8', 'a') and a[0] == 'mem':
                return [0x32] + w16(v(a[1]))
            if a == ('r8', 'a') and b[0] == 'ir':
                return [0xED, 0x57 if b[1] == 'i' else 0x5F]
            if a[0] == 'ir' and b == ('r8', 'a'):
                return [0xED, 0x47 if a[1] == 'i' else 0x4F]
            # 16 bit
            if a[0] == 'r16' and a[1] in RP and b[0] == 'imm':
                return [0x01 | RP[a[1]] << 4] + w16(v(b[1]))
            if a[0] == 'ix' and b[0] == 'imm':
                return [IDX[a[1]], 0x21] + w16(v(b[1]))
            if a == ('r16', 'hl') and b[0] == 'mem':
                return [0x2A] + w16(v(b[1]))
            if a[0] == 'r16' and a[1] in RP and b[0] == 'mem':
                return [0xED, 0x4B | RP[a[1]] << 4] + w16(v(b[1]))
            if a[0] == 'ix' and b[0] == 'mem':
                return [IDX[a[1]], 0x2A] + w16(v(b[1]))
            if a[0] == 'mem' and b == ('r16', 'hl'):
                return [0x22] + w16(v(a[1]))
            if a[0] == 'mem' and b[0] == 'r16' and b[1] in RP:
                return [0xED, 0x43 | RP[b[1]] << 4] + w16(v(a[1]))
            if a[0] == 'mem' and b[0] == 'ix':
                return [IDX[b[1]], 0x22] + w16(v(a[1]))
            if a == ('r16', 'sp') and b == ('r16', 'hl'):
                return [0xF9]
            if a == ('r16', 'sp') and b[0] == 'ix':
                return [IDX[b[1]], 0xF9]
            raise AsmError('bad ld')

        if mn in ('push', 'pop'):
            o = ops[0]
            base = 0xC5 if mn == 'push' else 0xC1
            if o[0] == 'r16' and o[1] in RP2:
                return [base | RP2[o[1]] << 4]
            if o[0] == 'ix':
                return [IDX[o[1]], base | 0x20]
            raise AsmError('bad ' + mn)

        if mn == 'ex':
            a, b = ops
            if a == ('r16', 'de') and b == ('r16', 'hl'):
                return [0xEB]
            if a == ('r16', 'af') and b[0] == 'afp':
                return [0x08]
            if a == ('ind', 'sp') and b == ('r16', 'hl'):
                return [0xE3]
            if a == ('ind', 'sp') and b[0] == 'ix':
                return [IDX[b[1]], 0xE3]
            raise AsmError('bad ex')

        if mn in ALU:
            if n == 2 and ops[0] == ('r8', 'a'):
                o = ops[1]
            elif n == 2 and mn in ('add', 'adc', 'sbc'):
                o = None
            elif n == 1:
                o = ops[0]
            else:
                o = None
            if o is not None:
                if o[0] == 'imm':
                    return [0xC6 | ALU[mn] << 3, b8(v(o[1]))]
                p, r, d = reg_or_mem(o)
                return p + [0x80 | ALU[mn] << 3 | r] + d
            a, b = ops
            if mn == 'add' and a == ('r16', 'hl') and b[0] == 'r16' and b[1] in RP:
                return [0x09 | RP[b[1]] << 4]
            if mn in ('adc', 'sbc') and a == ('r16', 'hl') and b[0] == 'r16' and b[1] in RP:
                return [0xED, (0x4A if mn == 'adc' else 0x42) | RP[b[1]] << 4]
            if mn == 'add' and a[0] == 'ix':
                if b[0] == 'ix' and b[1] == a[1]:
                    return [IDX[a[1]], 0x29]
                if b[0] == 'r16' and b[1] in ('bc', 'de', 'sp'):
                    return [IDX[a[1]], 0x09 | RP[b[1]] << 4]
            raise AsmError('bad ' + mn)

        if mn in ('inc', 'dec'):
            o = ops[0]
            if o[0] == 'r16' and o[1] in RP:
                return [(0x03 if mn == 'inc' else 0x0B) | RP[o[1]] << 4]
            if o[0] == 'ix':
                return [IDX[o[1]], 0x23 if mn == 'inc' else 0x2B]
            p, r, d = reg_or_mem(o)
            return p + [(0x04 if mn == 'inc' else 0x05) | r << 3] + d

        if mn in ROT:
            p, r, d = reg_or_mem(ops[0])
            if p:
                return p + [0xCB] + d + [ROT[mn] << 3 | 6]
            return [0xCB, ROT[mn] << 3 | r]

        if mn in ('bit', 'res', 'set'):
            bit = v(ops[0][1])
            if not 0 <= bit <= 7:
                raise AsmError('bad bit number')
            base = {'bit': 0x40, 'res': 0x80, 'set': 0xC0}[mn]
            p, r, d = reg_or_mem(ops[1])
            if p:
                return p + [0xCB] + d + [base | bit << 3 | 6]
            return [0xCB, base | bit << 3 | r]

        if mn == 'jp':
            if n == 1:
                o = ops[0]
                if o == ('ind', 'hl'):
                    return [0xE9]
                if o[0] == 'idx' and v(o[2]) == 0:
                    return [IDX[o[1]], 0xE9]
                return [0xC3] + w16(v(o[1]))
            cc = args[0].lower()
            return [0xC2 | CC[cc] << 3] + w16(v(ops[1][1]))

        if mn in ('jr', 'djnz'):
            if n == 1:
                op = 0x18 if mn == 'jr' else 0x10
                t = v(ops[0][1])
            else:
                cc = args[0].lower()
                if cc not in ('nz', 'z', 'nc', 'c'):
                    raise AsmError('bad jr condition')
                op = 0x20 | CC[cc] << 3
                t = v(ops[1][1])
            e = t - (pc + 2)
            if final and not -128 <= e <= 127:
                raise AsmError('jump out of range (%d)' % e)
            return [op, e & 0xFF]

        if mn == 'call':
            if n == 1:
                return [0xCD] + w16(v(ops[0][1]))
            return [0xC4 | CC[args[0].lower()] << 3] + w16(v(ops[1][1]))

        if mn == 'ret':
            if n == 0:
                return [0xC9]
            return [0xC0 | CC[args[0].lower()] << 3]

        if mn == 'rst':
            p = v(ops[0][1])
            if p not in range(0, 0x40, 8):
                raise AsmError('bad rst')
            return [0xC7 | p]

        if mn == 'im':
            return [0xED, {0: 0x46, 1: 0x56, 2: 0x5E}[v(ops[0][1])]]

        if mn == 'in':
            if n == 1 and ops[0] == ('ind', 'c'):
                return [0xED, 0x70]
            a, b = ops
            if a == ('r8', 'a') and b[0] == 'mem':
                return [0xDB, b8(v(b[1]))]
            if b == ('ind', 'c'):
                if a[0] == 'r8':
                    return [0xED, 0x40 | R8[a[1]] << 3]
                if a[0] == 'imm' and a[1].lower() == 'f':
                    return [0xED, 0x70]
            raise AsmError('bad in')

        if mn == 'out':
            a, b = ops
            if a[0] == 'mem' and b == ('r8', 'a'):
                return [0xD3, b8(v(a[1]))]
            if a == ('ind', 'c') and b[0] == 'r8':
                return [0xED, 0x41 | R8[b[1]] << 3]
            if a == ('ind', 'c') and b[0] == 'imm' and v(b[1]) == 0:
                return [0xED, 0x71]
            raise AsmError('bad out')

        raise AsmError('unknown instruction %s' % mn)

    # one pass ----------------------------------------------------------
    def run(self, lines, final):
        pc = 0
        for no, (path, line) in enumerate(lines):
            text = self.strip_comment(line)
            if not text.strip():
                continue
            try:
                label, mn, rest = self.split_line(text)
                if mn in ('equ',):
                    if not label:
                        raise AsmError('equ without a name')
                    self.sym[label.lower()] = self.value(rest, pc, final)
                    continue
                if label:
                    if not final and label.lower() in self.sym and label.lower() not in self.equs:
                        raise AsmError('label defined twice: %s' % label)
                    self.sym[label.lower()] = pc
                    if not final:
                        self.equs.discard(label.lower())
                if not mn:
                    continue
                args = split_args(rest)
                start = pc
                if mn == 'org':
                    pc = self.value(rest, pc, final)
                    continue
                if mn in ('db', 'defb', 'defm'):
                    data = []
                    for a in args:
                        if len(a) >= 2 and a[0] == a[-1] and a[0] in '"\'' and len(a) != 3:
                            data += [ord(c) for c in a[1:-1]]
                        else:
                            data.append(self.value(a, pc, final) & 0xFF)
                elif mn in ('dw', 'defw'):
                    data = []
                    for a in args:
                        x = self.value(a, pc, final)
                        data += [x & 0xFF, (x >> 8) & 0xFF]
                elif mn in ('ds', 'defs'):
                    cnt = self.value(args[0], pc, True)
                    fill = self.value(args[1], pc, final) & 0xFF if len(args) > 1 else 0
                    data = [fill] * cnt
                elif mn == 'incbin':
                    name = args[0].strip('"\'')
                    data = list(open(os.path.join(os.path.dirname(path), name), 'rb').read())
                elif mn == 'end':
                    break
                else:
                    data = self.encode(mn, args, pc, final)
                if final:
                    for i, b in enumerate(data):
                        if pc + i in self.out:
                            raise AsmError('overlaps code at %04X' % (pc + i))
                        self.out[pc + i] = b
                    self.listing.append('%04X  %-12s %s' % (start, ' '.join('%02X' % b for b in data[:4]),
                                                            line.rstrip()))
                pc += len(data)
            except AsmError as e:
                raise AsmError('%s:%d: %s\n    %s' % (os.path.basename(path), self.lineno[no], e, line.strip()))

    @staticmethod
    def strip_comment(line):
        out, q = '', None
        for i, ch in enumerate(line):
            if q:
                out += ch
                if ch == q:
                    q = None
            elif ch == ';':
                break
            elif ch in '"' or (ch == "'" and not out.lower().rstrip().endswith('af')):
                q = ch
                out += ch
            else:
                out += ch
        return out

    @staticmethod
    def split_line(text):
        label = None
        m = re.match(r'^([A-Za-z_.][\w.]*):?', text) if not text[0].isspace() else None
        if m:
            label = m.group(1)
            text = text[m.end():]
        parts = text.strip().split(None, 1)
        if not parts:
            return label, None, ''
        mn = parts[0].lower()
        rest = parts[1] if len(parts) > 1 else ''
        # "name equ value" with the name in the first column
        if label is None and len(parts) > 1 and parts[1].lower().startswith('equ '):
            return parts[0], 'equ', parts[1][4:]
        return label, mn, rest

    def read(self, path, lines):
        for i, l in enumerate(open(path, encoding='latin-1').read().splitlines()):
            m = re.match(r'\s+include\s+["\'](.+?)["\']', self.strip_comment(l), re.I)
            if m:
                self.read(os.path.join(os.path.dirname(path), m.group(1)), lines)
                continue
            lines.append((path, l))
            self.lineno.append(i + 1)

    def assemble(self, path):
        lines = []
        self.lineno = []
        self.equs = set()
        self.read(path, lines)
        self.run(lines, False)
        self.equs = set(self.sym)
        self.run(lines, True)


def write_outputs(data, out, opts):
    open(out, 'wb').write(data)
    if '--hex' in opts:
        open(opts['--hex'], 'w', newline='\n').write(''.join('%02X\n' % b for b in data))
    if '--mif' in opts:
        with open(opts['--mif'], 'w', newline='\n') as f:
            f.write('WIDTH=8;\nDEPTH=%d;\nADDRESS_RADIX=HEX;\nDATA_RADIX=HEX;\nCONTENT BEGIN\n' % len(data))
            for i, b in enumerate(data):
                f.write('\t%04X : %02X;\n' % (i, b))
            f.write('END;\n')


def main(argv):
    src, out = argv[1], argv[2]
    opts = dict(zip(argv[3::2], argv[4::2]))
    a = Asm()
    try:
        a.assemble(src)
    except AsmError as e:
        sys.exit(str(e))
    size = int(opts.get('--size', '0'), 0) or (max(a.out) + 1 if a.out else 0)
    if a.out and max(a.out) >= size:
        sys.exit('code goes past %04X' % size)
    data = bytes(a.out.get(i, 0) for i in range(size))
    write_outputs(data, out, opts)
    if '--list' in opts:
        open(opts['--list'], 'w', newline='\n').write('\n'.join(a.listing) + '\n')
    print('%s: %d bytes used of %d' % (os.path.basename(out), len(a.out), size))


if __name__ == '__main__':
    main(sys.argv)
