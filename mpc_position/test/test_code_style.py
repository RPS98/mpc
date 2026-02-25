#!/usr/bin/env python3

# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Test code style."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import subprocess


def run_flake8():
    """Run flake8 and return the output."""
    result = subprocess.run(
        ['ament_flake8'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    return result


def run_pep257():
    """Run pep257 and return the output."""
    result = subprocess.run(
        ['ament_pep257'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    return result


def run_pydocstyle():
    """Run pydocstyle and return the output."""
    result = subprocess.run(
        ['pydocstyle', '--convention=pep257', '--add-ignore=D104,D105'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    return result


def test_flake8():
    """Test that code conforms to PEP8."""
    result = run_flake8()
    assert result.returncode == 0, f"Flake8 found issues:\n{result.stdout.decode('utf-8')}"


def test_pep257():
    """Test that code conforms to docstring conventions."""
    result = run_pep257()
    assert result.returncode == 0, f"pep257 found issues:\n{result.stdout.decode('utf-8')}"


def test_pydocstyle():
    """Test that code conforms to docstring conventions."""
    result = run_pydocstyle()
    assert result.returncode == 0, f"pydocstyle found issues:\n{result.stdout.decode('utf-8')}"
