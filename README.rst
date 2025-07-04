pymodaq_plugins_cellkraft
#########################

.. the following must be adapted to your developed package, links to pypi, github  description...

.. image:: https://img.shields.io/pypi/v/pymodaq_plugins_cellkraft.svg
   :target: https://pypi.org/project/pymodaq_plugins_cellkraft/
   :alt: Latest Version

.. image:: https://readthedocs.org/projects/pymodaq/badge/?version=latest
   :target: https://pymodaq.readthedocs.io/en/stable/?badge=latest
   :alt: Documentation Status

.. image:: https://github.com/PyMoDAQ/pymodaq_plugins_cellkraft/workflows/Upload%20Python%20Package/badge.svg
   :target: https://github.com/PyMoDAQ/pymodaq_plugins_cellkraft
   :alt: Publication Status

.. image:: https://github.com/PyMoDAQ/pymodaq_plugins_cellkraft/actions/workflows/Test.yml/badge.svg
    :target: https://github.com/PyMoDAQ/pymodaq_plugins_cellkraft/actions/workflows/Test.yml


Plugin dedicated to Cellkraft devices

Authors
=======

* Loïc Guillmard  (loic.guillmard@cnrs.fr)
* Sébastien Guerrero  (sebastien.guerrero@insa-lyon.fr)
* Fabien Villedieu (fabien.villedieu.pro@gmail.com)

Instruments
===========

Below is the list of instruments included in this plugin
CellKraft ESeries

Actuators
+++++++++

* **daq_move_CellkraftE1500**: Control all the various parameters of the CellkraftE1500 steam generator

Viewer0D
++++++++

* **daq_0Dviewer_Pressure.py**: Acquires pressure from CellkraftE1500 steam generator

Configuration File
++++++++++++++++++
| You can configure the plugin with the toml file that is in user/documents/.pymodaq.
| You just have to fill the folder with this template that you modify :

.. code:: toml

    title = "Configuration file of the Cellkraft plugin"

    [Cellkraft.DEVICE01]
    name = "CellkraftE1500"
    host = "address_for_the_tcp_modbus"
