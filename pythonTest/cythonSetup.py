# setup.py
from setuptools import setup
from Cython.Build import cythonize

setup(
    ext_modules=cythonize([
        "SmoothBno085.pyx",
        "VL53L1XSensor.pyx",
        "RaspGSCamera.pyx",
        "PythonRelatedInitialization.pyx",
    ], compiler_directives={'language_level': "3"})
)