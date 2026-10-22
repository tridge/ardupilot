# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Select live aerobatics workers individually and with all three loaded."""

import importlib.util
import os
import shutil

import pytest

from test_aerobatics_flight import AerobaticVehicle
from test_aerobatics_path import APPLET

import build


def applet_builder(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.build


@pytest.fixture(scope='module')
def language_images(tmp_path_factory):
    if shutil.which(os.environ.get('ADA', 'llvm-gnat1')) is None or \
            shutil.which(os.environ.get('FC', 'gfortran')) is None:
        pytest.skip('Requires GNAT LLVM and GNU Fortran')
    directory = tmp_path_factory.mktemp('aerobatics-languages')
    ada = applet_builder('aerobatics_ada_builder', APPLET / 'ada/build_airshow.py')
    fortran = applet_builder('aerobatics_f90_builder', APPLET / 'f90/build.py')
    images = {}
    target = 'x86_64-unknown-linux-gnu'
    images[0] = build.build(APPLET / 'main.rs', directory / 'rust.apmod', target, 16384, heap_size=32768)
    previous = os.environ.get('ADA_INCLUDE_PATH')
    try:
        if 'ADA_NATIVE_INCLUDE_PATH' in os.environ:
            os.environ['ADA_INCLUDE_PATH'] = os.environ['ADA_NATIVE_INCLUDE_PATH']
        images[1] = ada(APPLET / 'ada/aerobatics.adb', directory / 'ada.apmod', target, 16384, heap_size=4096)
    finally:
        if previous is None:
            os.environ.pop('ADA_INCLUDE_PATH', None)
        else:
            os.environ['ADA_INCLUDE_PATH'] = previous
    images[2] = fortran(directory / 'f90.apmod', target)
    return images


@pytest.mark.parametrize('loaded', [(0,), (1,), (2,), (0, 1, 2)])
def test_language_selection(tmp_path, language_images, loaded):
    files = {language_images[i].name: language_images[i].read_bytes() for i in loaded}
    vehicle = AerobaticVehicle(tmp_path / 'vehicle', files, {
        'SCR_ENABLE': 0, 'MOD_HEAP_MAX': 32768, 'MOD_CODE_MAXSIZE': 262144,
        'MOD_STACK_MAX': 32768, 'RC7_OPTION': 300, 'RC9_OPTION': 301,
        'RC_OVERRIDE_TIME': 3, 'FLTMODE_CH': 0,
    })
    scripts = vehicle.directory / 'scripts'
    scripts.mkdir()
    shutil.copyfile(APPLET / 'AirShow.txt', scripts / 'trick72.txt')
    try:
        vehicle.start()
        assert vehicle.parameter('AEROM_LANG') == 0
        vehicle.parameter('TRIK_ENABLE', 1)
        vehicle.restart(1, expected=len(files))
        vehicle.parameter('TRIK1_ID', 72)
        vehicle.mode('CRUISE')
        workers = {name: vehicle.worker(name) for name in files}
        assert all(worker is not None for worker in workers.values())
        previous_language = 0
        names = {0: 'RUST', 1: 'ADA', 2: 'F90'}
        for language in (3, 0, 0, 1, 1, 2, 2, -1, 2, 1, 0):
            language_mark = len(vehicle.texts)
            assert vehicle.parameter('AEROM_LANG', language) == language
            vehicle.rc(7, 1000)
            vehicle.pump(0.5)
            mark = len(vehicle.texts)
            vehicle.rc(7, 1500)
            if language in loaded:
                vehicle.text('Trick 1 selected (SuperAirShow)', after=mark)
            vehicle.pump(1)
            selections = [text for text in vehicle.texts[mark:] if 'Trick 1 selected (SuperAirShow)' in text]
            assert len(selections) == int(language in loaded), (language, loaded, vehicle.diagnostic())
            announcements = [text for text in vehicle.texts[language_mark:]
                             if text.startswith('Aerobatics ') and text.endswith(' selected')]
            expected = []
            if language in loaded and language != previous_language:
                expected = [f'Aerobatics {names[language]} selected']
            assert announcements == expected, (language, loaded, announcements, vehicle.diagnostic())
            previous_language = language
            assert all(vehicle.worker(name)[0] == worker[0] for name, worker in workers.items())
        assert not any('panic' in text or 'returned -' in text or 'Aerobatics error' in text
                       for text in vehicle.texts), vehicle.diagnostic()
        vehicle.restart(2, expected=len(files))
        assert vehicle.parameter('AEROM_LANG') == 0
    finally:
        vehicle.stop()
