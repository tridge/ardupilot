# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later

import copy
import json

import pytest

import generate


def specification():
    return json.loads((generate.ROOT / 'generator/host_api.json').read_text())


def test_generated_files_match():
    outputs = generate.generate(specification())
    for name, content in outputs.items():
        assert (generate.ROOT / name).read_text() == content
    assert outputs == generate.generate(specification())


def test_fingerprint_changes_with_contract():
    original = specification()
    expected = generate.generate(original)['sdk/abi.json']
    for change in ('version', 'status', 'signature', 'order'):
        spec = copy.deepcopy(original)
        if change == 'version':
            spec['version'] += 1
        elif change == 'status':
            spec['status']['STOPPED'] += 1
        elif change == 'signature':
            spec['functions'][0]['args'][1][1] = 'u64'
        else:
            spec['functions'].reverse()
        assert generate.generate(spec)['sdk/abi.json'] != expected


def test_appended_host_callback_preserves_core_identity():
    spec = specification()
    before = generate.generate(spec)['sdk/abi.json']
    spec['functions'].append(dict(name='future', return_='void', args=[['context', 'pointer']]))
    spec['functions'][-1]['return'] = spec['functions'][-1].pop('return_')
    assert generate.generate(spec)['sdk/abi.json'] == before


@pytest.mark.parametrize('bad', ['duplicate', 'void_argument', 'missing_context', 'return_type', 'identifier'])
def test_reject_invalid_specification(bad):
    spec = specification()
    if bad == 'duplicate':
        spec['functions'].append(copy.deepcopy(spec['functions'][0]))
    elif bad == 'void_argument':
        spec['functions'][0]['args'][1][1] = 'void'
    elif bad == 'missing_context':
        spec['functions'][0]['args'].pop(0)
    elif bad == 'return_type':
        spec['functions'][0]['return'] = 'String'
    else:
        spec['functions'][0]['name'] = 'not a name'
    with pytest.raises(ValueError):
        generate.generate(spec)
