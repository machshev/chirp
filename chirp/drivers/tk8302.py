# Copyright 2023 Dan Smith <chirp@f.danplanet.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import functools
import logging
import struct

from chirp import bitwise
from chirp import chirp_common
from chirp import directory
from chirp.drivers import tk8180
from chirp import errors
from chirp import memmap
from chirp import settings
from chirp import util

LOG = logging.getLogger(__name__)

BLOCK_SIZE = 0x20
XOR_KEY    = 0x44
ACK        = b'\xbd'
NAK        = b'\xbe'

CMD_READ    = 0xe9
RESP_DATA   = 0xec
RESP_EMPTY  = 0xe1
RESP_IDENT  = 0xed

def xor_block(data, key=XOR_KEY):
    return bytes(b ^ key for b in data)


def send(radio, data):
    LOG.debug("TX: %s" % data.hex())
    radio.pipe.write(data)


def read_exact(radio, n):
    buf = b''
    while len(buf) < n:
        chunk = radio.pipe.read(n - len(buf))
        if not chunk:
            raise errors.RadioError(
                'Timeout reading %d bytes (got %d)' % (n, len(buf)))
        buf += chunk
    return buf


def expect_ack(radio):
    b = radio.pipe.read(1)
    if b != ACK:
        raise errors.RadioError(
            'Expected ACK 0xbd, got %r' % b)


def send_ack(radio):
    send(radio, ACK)
    expect_ack(radio)


def do_ident(radio):
    radio.pipe.baudrate = 9600
    radio.pipe.stopbits = 2
    radio.pipe.timeout = 1
    radio.pipe.dtr = True
    radio.pipe.rts = False

    send(radio, b'PROGRAM')
    ack = radio.pipe.read(1)
    if ack != b'\x16':
        raise errors.RadioError('Radio refused hi-speed program mode')

    radio.pipe.baudrate = 19200
    ack = radio.pipe.read(1)
    if ack != b'\x06':
        raise errors.RadioError('Radio refused program mode')

    radio.pipe.write(b'\x02')
    ident = radio.pipe.read(8)
    LOG.debug('Radio ident is %r', ident)
    meta = radio.pipe.read(32)
    LOG.debug('Radio meta is %r', meta)

    radio.pipe.write(b'\x22')
    radio.pipe.write(b'\x06')
    ack = radio.pipe.read(1)
    if ack != b'\x06':
        raise errors.RadioError('Radio refused program mode')

    if ident != radio._model:
        raise errors.RadioError('Unsupported radio model "%s"' % ident)

    # Not sure what this is yet?
    radio.pipe.write(b'\xeb')
    meta2 = radio.pipe.read(10)
    LOG.debug('Radio meta2 is %r', meta2)
    send_ack(radio)
    

def make_read_frame(addr):
    addr_hi = (addr >> 8) & 0xff
    addr_lo = addr & 0xff
    # Checksum observed as 0x9b throughout all known-good frames
    checksum = 0x9b
    frame = struct.pack("BBBB", CMD_READ, addr_hi, addr_lo, checksum)
    LOG.debug('Frame for 0x%04x: %s' % (addr, frame.hex()))
    return frame


def do_download(radio):
    do_ident(radio)

    data = bytearray()

    mem_start = radio._memstart
    mem_end   = radio._memend
    memsize  = mem_end - mem_start

    def status():
        st = chirp_common.Status()
        st.cur = len(data)
        st.max = memsize
        st.msg = "Cloning from radio"
        radio.status_fn(st)

    addr = mem_start
    while addr < mem_end:
        LOG.debug('Reading block at 0x%04x' % addr)

        send(radio, make_read_frame(addr))

        resp = radio.pipe.read(1)
        if not resp:
            raise errors.RadioError(
                'Timeout waiting for response at 0x%04x' % addr)

        opcode = resp[0]

        if opcode == RESP_EMPTY:
            # e1 <addr_hi> <addr_lo> <checksum> <0x44> — empty block
            radio.pipe.read(4)
            data += bytes(BLOCK_SIZE)
            LOG.debug('  Empty block at 0x%04x' % addr)

        elif opcode == RESP_DATA:
            radio.pipe.read(3)
            raw = read_exact(radio, BLOCK_SIZE)

            # Non-blocking peek — only wait 50ms for extra bytes
            radio.pipe.timeout = 0.05
            extra = radio.pipe.read(BLOCK_SIZE)
            radio.pipe.timeout = radio.BAUDS[radio._baud_rate] if hasattr(radio, 'BAUDS') else 1.0

            if extra:
                LOG.debug('  Extended block (+%d bytes) at 0x%04x' % (len(extra), addr))
                raw += extra

            block = xor_block(raw)
            data += block
            LOG.debug('  Data block at 0x%04x: %s' % (addr, block.hex()))

        elif opcode == NAK:
            raise errors.RadioError(
                'Radio NAKed request at 0x%04x (bad checksum/frame?)' % addr)

        else:
            raise errors.RadioError(
                'Unexpected opcode 0x%02x at 0x%04x' % (opcode, addr))

        status()
        send_ack(radio)
        addr += BLOCK_SIZE

    # End of clone
    LOG.debug('Sending end-of-clone 0xfe')
    send(radio, b'\xfe')
    ack = radio.pipe.read(1)
    if ack != ACK:
        raise errors.RadioError(
            'Radio did not acknowledge end of clone (got %r)' % ack)

    LOG.debug('Read %d bytes total' % len(data))
    return bytes(data)


@directory.register
class TK8302Radio(chirp_common.CloneModeRadio):
    VENDOR = 'Kenwood'
    MODEL = 'TK-8302'
    VALID_BANDS = [(400000000, 520000000)]
    FORMATS = [directory.register_format('Kenwood KPG-124D', '*.dat')]

    _model = b'M8302"3 '
    _memstart = 0xa400
    _memend   = 0xbffb
    _power_levels = [
        chirp_common.PowerLevel('Low', watts=5),
        chirp_common.PowerLevel('High', watts=25)]


    def sync_in(self):
        """Download from radio."""
        try:
            data = do_download(self)
            self._mmap = memmap.MemoryMap(data)
        except errors.RadioError:
            raise
        except Exception as e:
            LOG.exception('Failed download: %s' % e)
            raise errors.RadioError('Failed to communicate with radio')

        
    def get_features(self):
        rf = chirp_common.RadioFeatures()
        rf.memory_bounds = (1, 16)
        rf.has_ctone = True
        rf.has_cross = True
        rf.has_bank = False
        rf.has_sub_devices = False
        rf.has_dynamic_subdevices = rf.has_sub_devices
        rf.has_tuning_step = False
        rf.has_rx_dtcs = True
        rf.has_settings = True
        rf.can_odd_split = True
        rf.valid_tmodes = ['', 'Tone', 'TSQL', 'DTCS', 'Cross']
        rf.valid_cross_modes = ['Tone->Tone', 'DTCS->', '->DTCS', "Tone->DTCS",
                                'DTCS->Tone', '->Tone', 'DTCS->DTCS']
        rf.valid_duplexes = ['', '-', '+', 'split', 'off']
        rf.valid_modes = ['FM', 'NFM']
        rf.valid_tuning_steps = [2.5, 5.0, 6.25, 12.5, 10.0, 15.0, 20.0,
                                 25.0, 50.0, 100.0]
        rf.valid_bands = self.VALID_BANDS
        rf.valid_characters = chirp_common.CHARSET_UPPER_NUMERIC
        rf.valid_name_length = 8
        rf.valid_power_levels = list(reversed(self._power_levels))
        rf.valid_skips = ['', 'S']
        return rf
