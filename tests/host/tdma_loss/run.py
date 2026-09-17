#!/usr/bin/env python3
"""Exercise production sequence/recovery bodies, optionally from a pre-change tree."""

import argparse
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile


CASES = (
    'fast-recovery',
    'slow-probe',
    'sparse-recovery',
    'idle-breaks-evidence',
    'loss-blocks-probe',
    'threshold-boundaries',
    'counter-reset',
    'counter-wrap',
    'restart-breaks-evidence',
    'ping-restart-sequence',
    'sequence-wrap',
    'pending-publication',
    'excluded-modes',
)


def block(text, start):
    # Keep offsets intact while ignoring braces in comments and literals.
    non_code = re.compile(
        r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',
        re.DOTALL,
    )
    code = non_code.sub(lambda match: re.sub(r'[^\n]', ' ', match.group()), text)
    begin = code.index('{', start)
    depth = 0
    for token in re.finditer(r'[{}]', code[begin:]):
        depth += 1 if token.group() == '{' else -1
        if depth == 0:
            return text[start:begin + token.end()] + '\n'
    raise ValueError('Unclosed production block')


def function(source, name, optional=False):
    pattern = r'^[\w \t*]+\b' + re.escape(name) + r'\s*\([^;{}]*\)\s*\{'
    match = re.search(pattern, source, re.MULTILINE)
    if match is not None:
        return block(source, match.start())
    if optional:
        return ''
    raise ValueError(f'Missing production function: {name}')


def production_bodies(source):
    # Keep the selected tree's constants and state names together. Older trees
    # use *_WINDOWS / *_window_*; current code uses *_TICKS / *_pool_*.
    constants_start = source.index('static const uint16_t tdma_cap_ladder')
    constants_end = source.index('#define TDMA_SYNC_EXTRAP_MAX_TICKS')
    constants = source[constants_start:constants_end]
    declarations = '\n'.join(re.findall(
        r'^static (?:uint8_t|uint32_t) '
        r'tdma_(?:cap_level|published_cap_level|loss_\w+|prev_\w+)[^;]*;',
        source,
        re.MULTILINE,
    ))
    structure_start = re.search(r'\bstruct\s+packet_stats\s*\{', source)
    if structure_start is None:
        raise ValueError('Missing production packet_stats structure')
    structure = block(source, structure_start.start()) + ';\n'

    restart_start = re.search(r'\bif\s*\(\s*is_tracker_restart\s*\)\s*\{', source)
    if restart_start is None:
        raise ValueError('Missing production tracker restart branch')
    restart = block(source, restart_start.start())
    restart_wrapper = '\n'.join((
        'static void restart_ping(uint8_t tracker_id)',
        '{',
        '    bool is_tracker_restart = true;',
        '    uint8_t counter = 0;',
        '    uint64_t current_time = now;',
        restart,
        '}',
    ))

    return '\n'.join((
        constants,
        declarations,
        structure,
        'static struct packet_stats tracker_stats[MAX_TRACKERS];',
        # The pre-recovery-fix baseline has no shared reset helper.
        function(source, 'tdma_loss_reset_windows', optional=True),
        function(source, 'tdma_loss_controller_tick'),
        function(source, 'check_packet_sequence'),
        restart_wrapper,
    ))


def parse_args():
    default_root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--source-root',
        type=Path,
        default=Path(os.environ.get('SOURCE_ROOT', default_root)),
    )
    parser.add_argument('--case', action='append')
    return parser.parse_args()


def main():
    args = parse_args()
    source = (args.source_root / 'src/connection/esb.c').read_text()
    fixture = Path(__file__).with_name('fixture.c').read_text()
    prelude, scenarios = fixture.split('/* PRODUCTION BODIES */')

    with tempfile.TemporaryDirectory(prefix='receiver-tdma-loss-') as directory:
        path = Path(directory)
        generated = path / 'test.c'
        generated.write_text(prelude + production_bodies(source) + scenarios)
        binary = path / 'test'
        compiler = shlex.split(os.environ.get('CC', 'cc'))
        flags = [
            '-std=c11',
            '-Wall',
            '-Wextra',
            '-Werror',
            '-Wno-unused-function',
            '-Wno-unused-variable',
            '-Wno-unused-parameter',
        ]
        subprocess.run(compiler + flags + [str(generated), '-o', str(binary)], check=True)
        for case in args.case or CASES:
            subprocess.run([str(binary), case], check=True)
            print(f'PASS {case}', flush=True)


if __name__ == '__main__':
    main()
