=========
mini_afsd
=========

mini_afsd is a program for controlling a miniturized additive friction stir deposition (AFSD) machine.

* For Python 3.7+
* Open Source: BSD 3-Clause License
* Source Code: https://github.com/RyTheGuy355/MiniAFSDCode


.. contents:: **Contents**
    :depth: 1


Introduction
------------

To be added.


Installation
------------

TODO: Figure out how to install from private repository for releases and main branch.

Probably want to introduce how to install python using python's website, winpython, or anaconda.


Dependencies
~~~~~~~~~~~~

mini_afsd requires `Python <https://python.org>`_ version 3.7 or later
and the following Python libraries:

* `pyserial <https://pypi.org/project/pyserial/>`_
* `labjack-ljm <https://pypi.org/project/labjack-ljm/>`_


All of the required Python libraries should be automatically installed when
installing mini_afsd using any of the installation methods below.

The `LJM` driver from LabJack must also be installed, which can be downloaded from
https://labjack.com/support/software/installers/ljm.


Installing Python
~~~~~~~~~~~~~~~~~

Python can be installed multiple ways:

* If on Windows, the easiest way is to use `WinPython <https://winpython.github.io/>`_. The recommended
  installation file (as of June 10, 2022) is WinPython64-3.10.4.0 (or WinPython64-3.10.4.0dot if you don't
  want any preinstalled libraries).
* Use `Anaconda <https://www.anaconda.com/>`_, which comes with many libraries preinstalled.
* Install from Python's official source, https://www.python.org/. Follow the instructions listed at
  https://packaging.python.org/en/latest/tutorials/installing-packages/#requirements-for-installing-packages
  to ensure Python and the Python package manager `pip <https://pip.pypa.io>`_ are correctly installed.


Stable Release
~~~~~~~~~~~~~~

mini_afsd can be installed from `pypi <https://pypi.org/project/mini_afsd>`_
using `pip <https://pip.pypa.io>`_, by running the following command in the terminal:

.. code-block:: console

    pip install mini_afsd


Development Version
~~~~~~~~~~~~~~~~~~~

The sources for mini_afsd can be downloaded from the `GitHub repo`_.
To install the current version of mini_afsd from GitHub, run:

.. code-block:: console

    pip install https://github.com/RyTheGuy355/MiniAFSDCode/zipball/main


.. _GitHub repo: https://github.com/RyTheGuy355/MiniAFSDCode


Quick Start
-----------

To be added.


Contributing
------------

Contributions are welcomed and greatly appreciated. For information on
submitting bug reports, pull requests, or general feedback, please refer
to the `contributing guide`_.

.. _contributing guide: https://github.com/RyTheGuy355/MiniAFSDCode/tree/main/docs/contributing.rst


Changelog
---------

Refer to the changelog_ for information on mini_afsd's changes.

.. _changelog: https://github.com/RyTheGuy355/MiniAFSDCode/tree/main/CHANGELOG.rst


License
-------

mini_afsd is all rights reserved. For more information, refer to the license_.

.. _license: https://github.com/RyTheGuy355/MiniAFSDCode/tree/main/LICENSE.txt


Author
------

* Ryan Gottwald <insert_email_here>
